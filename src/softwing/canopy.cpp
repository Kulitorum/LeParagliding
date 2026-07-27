#include "softwing/canopy.h"

#include <algorithm>
#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <numbers>
#include <set>
#include <sstream>
#include <tuple>
#include <unordered_map>
#include <utility>

namespace softwing {
namespace {

constexpr std::size_t kMaximumCanonicalCount = 1'000'000;
constexpr std::size_t kMaximumCanonicalBytes = 64U * 1024U * 1024U;
constexpr double kReferenceDensity = 0.05;
constexpr std::string_view kFrameTag =
    "SI_RH_X_LE_TO_TE_Y_SPAN_Z_UP";

[[noreturn]] void fail(CanopyPhase phase,
                       const std::string& entity,
                       const std::string& message) {
    throw CanopyError(phase, entity, message);
}

bool finite(double value) { return std::isfinite(value); }

bool validUtf8(std::string_view value) {
    auto continuation = [](unsigned char byte) {
        return byte >= 0x80U && byte <= 0xBFU;
    };
    for (std::size_t i = 0; i < value.size();) {
        const unsigned char first = static_cast<unsigned char>(value[i]);
        if (first <= 0x7FU) {
            ++i;
            continue;
        }
        if (first >= 0xC2U && first <= 0xDFU) {
            if (i + 1 >= value.size() ||
                !continuation(static_cast<unsigned char>(value[i + 1]))) {
                return false;
            }
            i += 2;
            continue;
        }
        if (first >= 0xE0U && first <= 0xEFU) {
            if (i + 2 >= value.size()) {
                return false;
            }
            const unsigned char second = static_cast<unsigned char>(value[i + 1]);
            const unsigned char third = static_cast<unsigned char>(value[i + 2]);
            const bool validSecond =
                first == 0xE0U ? second >= 0xA0U && second <= 0xBFU
                : first == 0xEDU ? second >= 0x80U && second <= 0x9FU
                                 : continuation(second);
            if (!validSecond || !continuation(third)) {
                return false;
            }
            i += 3;
            continue;
        }
        if (first >= 0xF0U && first <= 0xF4U) {
            if (i + 3 >= value.size()) {
                return false;
            }
            const unsigned char second = static_cast<unsigned char>(value[i + 1]);
            const bool validSecond =
                first == 0xF0U ? second >= 0x90U && second <= 0xBFU
                : first == 0xF4U ? second >= 0x80U && second <= 0x8FU
                                 : continuation(second);
            if (!validSecond ||
                !continuation(static_cast<unsigned char>(value[i + 2])) ||
                !continuation(static_cast<unsigned char>(value[i + 3]))) {
                return false;
            }
            i += 4;
            continue;
        }
        return false;
    }
    return true;
}

void requireUtf8(std::string_view value,
                 CanopyPhase phase,
                 const std::string& entity,
                 const std::string& field) {
    if (!validUtf8(value)) {
        fail(phase, entity, field + " must be valid UTF-8 text");
    }
}

void requireFinite(double value,
                   CanopyPhase phase,
                   const std::string& entity,
                   const std::string& field) {
    if (!finite(value)) {
        fail(phase, entity, field + " must be finite");
    }
}

template <typename Record>
void requireUniqueIds(std::span<const Record> records,
                      const std::string& entity) {
    std::set<std::string> ids;
    for (const Record& record : records) {
        if (record.id.empty()) {
            fail(CanopyPhase::Validation, entity, "id must not be empty");
        }
        if (!ids.insert(record.id).second) {
            fail(CanopyPhase::Validation, record.id, "duplicate id");
        }
    }
}

bool hasProvenance(const CanopyDefinition& definition,
                   const std::string& id) {
    return std::ranges::any_of(definition.provenance,
                               [&](const CanopyProvenance& value) {
                                   return value.id == id;
                               });
}

const CanopyMaterial& materialById(const CanopyDefinition& definition,
                                   const std::string& id) {
    const auto found = std::ranges::find_if(
        definition.materials,
        [&](const CanopyMaterial& value) { return value.id == id; });
    if (found == definition.materials.end()) {
        fail(CanopyPhase::Validation, id, "unknown material reference");
    }
    return *found;
}

const CanopyPanelAssignment& assignmentFor(
    const CanopyDefinition& definition,
    CanopyPanelRole role) {
    const auto found = std::ranges::find_if(
        definition.assignments,
        [&](const CanopyPanelAssignment& value) { return value.role == role; });
    if (found == definition.assignments.end()) {
        fail(CanopyPhase::Validation,
             canopyPanelRoleName(role),
             "missing panel material assignment");
    }
    return *found;
}

double sectionOrdinate(const CanopySpanStation& station,
                       double u,
                       bool upper) {
    if (u <= station.section.front().u) {
        return upper ? station.section.front().upper
                     : station.section.front().lower;
    }
    for (std::size_t i = 1; i < station.section.size(); ++i) {
        const RibSectionSample& next = station.section[i];
        if (u <= next.u) {
            const RibSectionSample& previous = station.section[i - 1];
            const double fraction = (u - previous.u) / (next.u - previous.u);
            const double a = upper ? previous.upper : previous.lower;
            const double b = upper ? next.upper : next.lower;
            return a + fraction * (b - a);
        }
    }
    return upper ? station.section.back().upper : station.section.back().lower;
}

std::string encodeBytes(std::string_view value) {
    constexpr char digits[] = "0123456789ABCDEF";
    std::string encoded;
    encoded.reserve(value.size() * 2);
    for (const unsigned char byte : value) {
        encoded.push_back(digits[byte >> 4]);
        encoded.push_back(digits[byte & 0x0f]);
    }
    return encoded;
}

std::string decodeBytes(std::size_t byteCount,
                        std::string_view encoded,
                        const std::string& entity) {
    if (byteCount > kMaximumCanonicalCount || encoded.size() != byteCount * 2) {
        fail(CanopyPhase::Parse, entity, "invalid byte count");
    }
    auto nibble = [&](char value) -> unsigned char {
        if (value >= '0' && value <= '9') {
            return static_cast<unsigned char>(value - '0');
        }
        if (value >= 'A' && value <= 'F') {
            return static_cast<unsigned char>(value - 'A' + 10);
        }
        fail(CanopyPhase::Parse, entity, "invalid uppercase hexadecimal text");
    };
    std::string decoded;
    decoded.reserve(byteCount);
    for (std::size_t i = 0; i < encoded.size(); i += 2) {
        decoded.push_back(static_cast<char>((nibble(encoded[i]) << 4) |
                                            nibble(encoded[i + 1])));
    }
    requireUtf8(decoded, CanopyPhase::Parse, entity, "decoded value");
    return decoded;
}

void writeString(std::ostream& output,
                 std::string_view label,
                 std::string_view value) {
    output << label << ' ' << value.size() << ' ' << encodeBytes(value) << '\n';
}

std::vector<std::string> splitLines(std::string_view text) {
    std::vector<std::string> lines;
    std::size_t start = 0;
    while (start < text.size()) {
        const std::size_t end = text.find('\n', start);
        const std::size_t length = end == std::string_view::npos
                                       ? text.size() - start
                                       : end - start;
        std::string line{text.substr(start, length)};
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        lines.push_back(std::move(line));
        if (end == std::string_view::npos) {
            start = text.size();
        } else {
            start = end + 1;
        }
    }
    return lines;
}

class CanonicalReader {
public:
    explicit CanonicalReader(std::string_view text) {
        if (text.size() > kMaximumCanonicalBytes) {
            fail(CanopyPhase::Parse,
                 "document",
                 "canonical document exceeds the bounded input size");
        }
        lines_ = splitLines(text);
    }

    std::istringstream line(const std::string& expected) {
        if (cursor_ >= lines_.size()) {
            fail(CanopyPhase::Parse, expected, "truncated document");
        }
        std::istringstream input(lines_[cursor_++]);
        input.imbue(std::locale::classic());
        std::string label;
        if (!(input >> label) || label != expected) {
            fail(CanopyPhase::Parse, expected, "missing or out-of-order field");
        }
        return input;
    }

    std::string string(const std::string& expected) {
        std::istringstream input = line(expected);
        std::size_t size = 0;
        std::string encoded;
        if (!(input >> size >> encoded)) {
            fail(CanopyPhase::Parse, expected, "invalid string field");
        }
        requireEnd(input, expected);
        return decodeBytes(size, encoded, expected);
    }

    std::size_t count(const std::string& expected) {
        std::istringstream input = line(expected);
        std::size_t result = 0;
        if (!(input >> result) || result > kMaximumCanonicalCount) {
            fail(CanopyPhase::Parse, expected, "invalid or excessive count");
        }
        requireEnd(input, expected);
        claimCount(result, expected);
        return result;
    }

    void claimCount(std::size_t count, const std::string& entity) {
        if (count > remainingRecordBudget_) {
            fail(CanopyPhase::Parse,
                 entity,
                 "canonical document exceeds the total record budget");
        }
        remainingRecordBudget_ -= count;
    }

    void end() {
        std::istringstream input = line("END");
        requireEnd(input, "END");
        while (cursor_ < lines_.size() && lines_[cursor_].empty()) {
            ++cursor_;
        }
        if (cursor_ != lines_.size()) {
            fail(CanopyPhase::Parse, "trailing-data", "unexpected trailing data");
        }
    }

    static void requireEnd(std::istringstream& input,
                           const std::string& entity) {
        std::string trailing;
        if (input >> trailing) {
            fail(CanopyPhase::Parse, entity, "unexpected trailing field data");
        }
    }

private:
    std::vector<std::string> lines_;
    std::size_t cursor_ = 0;
    std::size_t remainingRecordBudget_ = kMaximumCanonicalCount;
};

template <typename Enum>
Enum checkedEnum(std::size_t value,
                 std::size_t count,
                 const std::string& entity) {
    if (value >= count) {
        fail(CanopyPhase::Parse, entity, "invalid enum value");
    }
    return static_cast<Enum>(value);
}

} // namespace

CanopyError::CanopyError(CanopyPhase phase,
                         std::string entity,
                         const std::string& message)
    : std::runtime_error(std::string(canopyPhaseName(phase)) + ": " + entity +
                         ": " + message),
      phase_(phase),
      entity_(std::move(entity)) {}

const char* canopyPhaseName(CanopyPhase phase) {
    switch (phase) {
    case CanopyPhase::Parse: return "parse";
    case CanopyPhase::Validation: return "validation";
    case CanopyPhase::Mapping: return "mapping";
    case CanopyPhase::Topology: return "topology";
    case CanopyPhase::Material: return "material";
    case CanopyPhase::Envelope: return "envelope";
    case CanopyPhase::Pneumatic: return "pneumatic";
    }
    return "invalid";
}

const char* canopyPanelRoleName(CanopyPanelRole role) {
    switch (role) {
    case CanopyPanelRole::UpperSkin: return "upper-skin";
    case CanopyPanelRole::LowerSkin: return "lower-skin";
    case CanopyPanelRole::LeadingEdge: return "leading-edge";
    case CanopyPanelRole::Rib: return "rib";
    case CanopyPanelRole::Diagonal: return "diagonal";
    }
    return "invalid";
}

std::size_t canopyPanelRoleIndex(CanopyPanelRole role) {
    switch (role) {
    case CanopyPanelRole::UpperSkin: return 0;
    case CanopyPanelRole::LowerSkin: return 1;
    case CanopyPanelRole::LeadingEdge: return 2;
    case CanopyPanelRole::Rib: return 3;
    case CanopyPanelRole::Diagonal: return 4;
    }
    throw std::invalid_argument("invalid canopy panel role");
}

const char* canopyApertureKindName(CanopyApertureKind kind) {
    switch (kind) {
    case CanopyApertureKind::Inlet: return "inlet";
    case CanopyApertureKind::CrossPort: return "cross-port";
    }
    return "invalid";
}

const char* canopyProvenanceName(CanopyValueProvenance value) {
    switch (value) {
    case CanopyValueProvenance::Prescribed: return "prescribed";
    case CanopyValueProvenance::Generated: return "generated";
    case CanopyValueProvenance::Derived: return "derived";
    case CanopyValueProvenance::Virtual: return "virtual";
    case CanopyValueProvenance::Residual: return "residual";
    }
    return "invalid";
}

bool CanopyAudit::valid() const {
    return unclassifiedStructuralFaces == 0 &&
           multiplyClassifiedStructuralFaces == 0 &&
           undeclaredBoundaryEdges == 0 &&
           undeclaredNonManifoldEdges == 0 && degenerateFaces == 0 &&
           coincidentDuplicateSeamNodes == 0;
}

CanopyDefinition validateAndNormalizeCanopyDefinition(
    const CanopyDefinition& input) {
    CanopyDefinition definition = input;
    if (definition.schemaMajor != 1 && definition.schemaMajor != 2) {
        fail(CanopyPhase::Validation,
             "schema-major",
             "only versions 1 and 2 are supported");
    }
    if (definition.identifier.empty() || definition.description.empty()) {
        fail(CanopyPhase::Validation, "definition", "identifier and description are required");
    }
    requireUtf8(definition.identifier,
                CanopyPhase::Validation,
                "definition",
                "identifier");
    requireUtf8(definition.description,
                CanopyPhase::Validation,
                "definition",
                "description");
    if (definition.unitsFrameTag != kFrameTag) {
        fail(CanopyPhase::Validation, "units-frame", "unsupported SI/frame tag");
    }
    if (definition.provenance.empty()) {
        fail(CanopyPhase::Validation, "provenance", "at least one source is required");
    }
    requireUniqueIds(std::span<const CanopyProvenance>{definition.provenance},
                     "provenance");
    for (const CanopyProvenance& provenance : definition.provenance) {
        requireUtf8(provenance.id,
                    CanopyPhase::Validation,
                    "provenance",
                    "id");
        requireUtf8(provenance.source,
                    CanopyPhase::Validation,
                    provenance.id,
                    "source");
        if (provenance.source.empty()) {
            fail(CanopyPhase::Validation, provenance.id, "source must not be empty");
        }
    }
    if (definition.cellCount == 0 ||
        definition.stations.size() != definition.cellCount + 1) {
        fail(CanopyPhase::Validation,
             "cell-count",
             "station count must equal cell count plus one");
    }
    requireUniqueIds(std::span<const CanopySpanStation>{definition.stations},
                     "station");
    for (std::size_t i = 0; i < definition.stations.size(); ++i) {
        const CanopySpanStation& station = definition.stations[i];
        requireUtf8(station.id,
                    CanopyPhase::Validation,
                    "station",
                    "id");
        requireFinite(station.y, CanopyPhase::Validation, station.id, "y");
        requireFinite(station.leadingEdgeX,
                      CanopyPhase::Validation,
                      station.id,
                      "leading edge x");
        requireFinite(station.arcZ, CanopyPhase::Validation, station.id, "arc z");
        requireFinite(station.chord, CanopyPhase::Validation, station.id, "chord");
        requireFinite(station.sectionRollRadians,
                      CanopyPhase::Validation,
                      station.id,
                      "section roll");
        requireFinite(station.twistRadians,
                      CanopyPhase::Validation,
                      station.id,
                      "twist");
        if (!(station.chord > 0.0)) {
            fail(CanopyPhase::Validation, station.id, "chord must be positive");
        }
        if (definition.schemaMajor == 1 && station.sectionRollRadians != 0.0) {
            fail(CanopyPhase::Validation,
                 station.id,
                 "version 1 requires zero section roll");
        }
        if (!(std::abs(station.sectionRollRadians) <
              canopySectionRollLimitRadians)) {
            fail(CanopyPhase::Validation,
                 station.id,
                 "section roll is outside the open frame bound");
        }
        if (i > 0 && !(station.y > definition.stations[i - 1].y)) {
            fail(CanopyPhase::Validation, station.id, "stations must be strictly ordered");
        }
        if (station.section.size() < 2 || station.section.front().u != 0.0 ||
            station.section.back().u != 1.0) {
            fail(CanopyPhase::Validation,
                 station.id,
                 "section samples must span exactly [0,1]");
        }
        for (std::size_t j = 0; j < station.section.size(); ++j) {
            const RibSectionSample& sample = station.section[j];
            requireFinite(sample.u, CanopyPhase::Validation, station.id, "section u");
            requireFinite(sample.upper,
                          CanopyPhase::Validation,
                          station.id,
                          "upper ordinate");
            requireFinite(sample.lower,
                          CanopyPhase::Validation,
                          station.id,
                          "lower ordinate");
            if (j > 0 && !(sample.u > station.section[j - 1].u)) {
                fail(CanopyPhase::Validation,
                     station.id,
                     "section samples must be strictly ordered");
            }
            const bool trailingEndpoint = j + 1 == station.section.size();
            if ((!trailingEndpoint && !(sample.upper > sample.lower)) ||
                (trailingEndpoint && sample.upper != sample.lower))
                fail(CanopyPhase::Validation,
                     station.id,
                     trailingEndpoint
                         ? "trailing section ordinates must be exactly equal"
                         : "section gap must be positive before the trailing edge");
        }
    }
    if (definition.materials.empty()) {
        fail(CanopyPhase::Validation, "materials", "material list is empty");
    }
    requireUniqueIds(std::span<const CanopyMaterial>{definition.materials},
                     "material");
    for (const CanopyMaterial& material : definition.materials) {
        requireUtf8(material.id,
                    CanopyPhase::Validation,
                    "material",
                    "id");
        requireUtf8(material.provenanceId,
                    CanopyPhase::Validation,
                    material.id,
                    "provenance id");
        requireFinite(material.arealDensity,
                      CanopyPhase::Validation,
                      material.id,
                      "areal density");
        if (!(material.arealDensity > 0.0)) {
            fail(CanopyPhase::Validation,
                 material.id,
                 "areal density must be positive");
        }
        try {
            validateOrthotropicMembraneMaterial(material.membrane);
        } catch (const std::exception& error) {
            fail(CanopyPhase::Validation, material.id, error.what());
        }
        if (!hasProvenance(definition, material.provenanceId)) {
            fail(CanopyPhase::Validation,
                 material.id,
                 "unknown material provenance");
        }
    }
    if (definition.assignments.size() != canopyPanelRoleCount) {
        fail(CanopyPhase::Validation,
             "assignments",
             "exactly five panel-role assignments are required");
    }
    std::array<bool, canopyPanelRoleCount> assigned{};
    for (const CanopyPanelAssignment& assignment : definition.assignments) {
        requireUtf8(assignment.materialId,
                    CanopyPhase::Validation,
                    "assignment",
                    "material id");
        std::size_t role = 0;
        try {
            role = canopyPanelRoleIndex(assignment.role);
        } catch (const std::invalid_argument&) {
            fail(CanopyPhase::Validation,
                 "assignments",
                 "duplicate or invalid panel role");
        }
        if (assigned[role]) {
            fail(CanopyPhase::Validation,
                 "assignments",
                 "duplicate or invalid panel role");
        }
        assigned[role] = true;
        static_cast<void>(materialById(definition, assignment.materialId));
    }
    requireFinite(definition.mesh.coordinateTolerance,
                  CanopyPhase::Validation,
                  "mesh",
                  "coordinate tolerance");
    requireFinite(definition.mesh.volumeFloor,
                  CanopyPhase::Validation,
                  "mesh",
                  "volume floor");
    if (definition.mesh.chordSubdivisions == 0 ||
        definition.mesh.spanSubdivisionsPerCell == 0 ||
        definition.mesh.thicknessSubdivisions == 0 ||
        definition.mesh.chordSubdivisions > 4096 ||
        definition.mesh.spanSubdivisionsPerCell > 4096 ||
        definition.mesh.thicknessSubdivisions > 4096 ||
        !(definition.mesh.coordinateTolerance > 0.0) ||
        !(definition.mesh.volumeFloor > 0.0)) {
        fail(CanopyPhase::Validation, "mesh", "invalid mesh settings");
    }
    requireUniqueIds(std::span<const CanopyApertureDefinition>{definition.apertures},
                     "aperture");
    for (const CanopyApertureDefinition& aperture : definition.apertures) {
        requireUtf8(aperture.id,
                    CanopyPhase::Validation,
                    "aperture",
                    "id");
        requireUtf8(aperture.provenanceId,
                    CanopyPhase::Validation,
                    aperture.id,
                    "provenance id");
        requireFinite(aperture.firstMin,
                      CanopyPhase::Validation,
                      aperture.id,
                      "first minimum");
        requireFinite(aperture.firstMax,
                      CanopyPhase::Validation,
                      aperture.id,
                      "first maximum");
        requireFinite(aperture.secondMin,
                      CanopyPhase::Validation,
                      aperture.id,
                      "second minimum");
        requireFinite(aperture.secondMax,
                      CanopyPhase::Validation,
                      aperture.id,
                      "second maximum");
        const double tolerance = definition.mesh.coordinateTolerance;
        if (!(aperture.firstMin >= 0.0 && aperture.firstMax <= 1.0 &&
              aperture.secondMin >= 0.0 && aperture.secondMax <= 1.0) ||
            !(aperture.firstMax - aperture.firstMin > tolerance) ||
            !(aperture.secondMax - aperture.secondMin > tolerance)) {
            fail(CanopyPhase::Validation,
                 aperture.id,
                 "aperture is out of panel bounds or below tolerance");
        }
        if (aperture.kind == CanopyApertureKind::Inlet) {
            if (aperture.cellIndex >= definition.cellCount) {
                fail(CanopyPhase::Validation, aperture.id, "invalid inlet cell");
            }
        } else if (aperture.kind == CanopyApertureKind::CrossPort) {
            if (aperture.ribIndex == 0 ||
                aperture.ribIndex >= definition.cellCount ||
                aperture.cellIndex + 1 != aperture.ribIndex) {
                fail(CanopyPhase::Validation,
                     aperture.id,
                     "invalid internal rib/cell association");
            }
        } else {
            fail(CanopyPhase::Validation, aperture.id, "unsupported aperture kind");
        }
        if (!hasProvenance(definition, aperture.provenanceId)) {
            fail(CanopyPhase::Validation,
                 aperture.id,
                 "unknown aperture provenance");
        }
    }
    for (std::size_t i = 0; i < definition.apertures.size(); ++i) {
        for (std::size_t j = i + 1; j < definition.apertures.size(); ++j) {
            const CanopyApertureDefinition& a = definition.apertures[i];
            const CanopyApertureDefinition& b = definition.apertures[j];
            const bool samePanel = a.kind == b.kind &&
                ((a.kind == CanopyApertureKind::Inlet && a.cellIndex == b.cellIndex) ||
                 (a.kind == CanopyApertureKind::CrossPort &&
                  a.ribIndex == b.ribIndex));
            if (!samePanel) {
                continue;
            }
            const bool separated =
                a.firstMax < b.firstMin || b.firstMax < a.firstMin ||
                a.secondMax < b.secondMin || b.secondMax < a.secondMin;
            if (!separated) {
                fail(CanopyPhase::Validation,
                     a.id,
                     "apertures overlap or touch on one panel");
            }
        }
    }
    requireUniqueIds(std::span<const CanopyDiagonalDefinition>{definition.diagonals},
                     "diagonal");
    std::set<std::string> panelIds;
    for (std::size_t cell = 0; cell < definition.cellCount; ++cell) {
        for (const CanopyPanelRole role :
             {CanopyPanelRole::UpperSkin,
              CanopyPanelRole::LowerSkin,
              CanopyPanelRole::LeadingEdge}) {
            panelIds.insert(std::string(canopyPanelRoleName(role)) + "-" +
                            std::to_string(cell));
        }
    }
    for (std::size_t rib = 0; rib <= definition.cellCount; ++rib) {
        panelIds.insert("rib-" + std::to_string(rib));
    }
    for (const CanopyDiagonalDefinition& diagonal : definition.diagonals) {
        requireUtf8(diagonal.id,
                    CanopyPhase::Validation,
                    "diagonal",
                    "id");
        requireUtf8(diagonal.materialId,
                    CanopyPhase::Validation,
                    diagonal.id,
                    "material id");
        requireUtf8(diagonal.provenanceId,
                    CanopyPhase::Validation,
                    diagonal.id,
                    "provenance id");
        if (!panelIds.insert(diagonal.id).second) {
            fail(CanopyPhase::Validation,
                 diagonal.id,
                 "diagonal id collides with another semantic panel id");
        }
        requireFinite(diagonal.chordMin,
                      CanopyPhase::Validation,
                      diagonal.id,
                      "chord minimum");
        requireFinite(diagonal.chordMax,
                      CanopyPhase::Validation,
                      diagonal.id,
                      "chord maximum");
        if (diagonal.cellIndex >= definition.cellCount ||
            !(diagonal.chordMin >= 0.0 && diagonal.chordMax <= 1.0) ||
            !(diagonal.chordMax - diagonal.chordMin >
              definition.mesh.coordinateTolerance)) {
            fail(CanopyPhase::Validation,
                 diagonal.id,
                 "invalid diagonal bounds or cell");
        }
        static_cast<void>(materialById(definition, diagonal.materialId));
        if (!hasProvenance(definition, diagonal.provenanceId)) {
            fail(CanopyPhase::Validation,
                 diagonal.id,
                 "unknown diagonal provenance");
        }
    }
    for (std::size_t i = 0; i < definition.diagonals.size(); ++i) {
        for (std::size_t j = i + 1; j < definition.diagonals.size(); ++j) {
            const CanopyDiagonalDefinition& a = definition.diagonals[i];
            const CanopyDiagonalDefinition& b = definition.diagonals[j];
            if (a.cellIndex != b.cellIndex) {
                continue;
            }
            if (!(a.chordMax < b.chordMin || b.chordMax < a.chordMin)) {
                fail(CanopyPhase::Validation,
                     a.id,
                     "diagonal webs overlap or touch in one cell");
            }
        }
    }
    auto strictlyInside = [](double value, double minimum, double maximum) {
        return value > minimum && value < maximum;
    };
    for (std::size_t i = 0; i < definition.apertures.size(); ++i) {
        const CanopyApertureDefinition& aperture = definition.apertures[i];
        for (std::size_t j = 0; j < definition.apertures.size(); ++j) {
            if (i == j) {
                continue;
            }
            const CanopyApertureDefinition& other = definition.apertures[j];
            if (strictlyInside(other.secondMin,
                               aperture.secondMin,
                               aperture.secondMax) ||
                strictlyInside(other.secondMax,
                               aperture.secondMin,
                               aperture.secondMax)) {
                fail(CanopyPhase::Validation,
                     aperture.id,
                     "aperture thickness bounds are incompatible with exact structured closure");
            }
            if (aperture.kind == CanopyApertureKind::CrossPort &&
                other.kind == CanopyApertureKind::CrossPort &&
                (strictlyInside(other.firstMin,
                                aperture.firstMin,
                                aperture.firstMax) ||
                 strictlyInside(other.firstMax,
                                aperture.firstMin,
                                aperture.firstMax))) {
                fail(CanopyPhase::Validation,
                     aperture.id,
                     "cross-port chord bounds are incompatible with exact structured closure");
            }
        }
        if (aperture.kind == CanopyApertureKind::CrossPort) {
            for (const CanopyDiagonalDefinition& diagonal :
                 definition.diagonals) {
                if (strictlyInside(diagonal.chordMin,
                                   aperture.firstMin,
                                   aperture.firstMax) ||
                    strictlyInside(diagonal.chordMax,
                                   aperture.firstMin,
                                   aperture.firstMax)) {
                    fail(CanopyPhase::Validation,
                         aperture.id,
                         "diagonal chord bounds are incompatible with exact cross-port closure");
                }
            }
        }
    }
    return definition;
}

CanopyStationFrame canopyStationFrame(const CanopySpanStation& station) {
    requireFinite(station.sectionRollRadians,
                  CanopyPhase::Mapping,
                  station.id,
                  "section roll");
    requireFinite(station.twistRadians,
                  CanopyPhase::Mapping,
                  station.id,
                  "twist");
    const double rollCosine = std::cos(station.sectionRollRadians);
    const double rollSine = std::sin(station.sectionRollRadians);
    const Vec3 nominalChord{1.0, 0.0, 0.0};
    const Vec3 span{0.0, rollCosine, rollSine};
    const Vec3 nominalUp{0.0, -rollSine, rollCosine};
    const double twistCosine = std::cos(station.twistRadians);
    const double twistSine = std::sin(station.twistRadians);
    const CanopyStationFrame frame{
        twistCosine * nominalChord - twistSine * nominalUp,
        span,
        twistSine * nominalChord + twistCosine * nominalUp};
    for (const Vec3 axis : {frame.chord, frame.span, frame.up}) {
        if (!finite(axis.x) || !finite(axis.y) || !finite(axis.z)) {
            fail(CanopyPhase::Mapping,
                 station.id,
                 "station frame is non-finite");
        }
    }
    return frame;
}

Vec3 mapCanopyStationPoint(const CanopySpanStation& station,
                           double u,
                           double zeta) {
    requireFinite(u, CanopyPhase::Mapping, station.id, "u");
    requireFinite(zeta, CanopyPhase::Mapping, station.id, "zeta");
    const CanopyStationFrame frame = canopyStationFrame(station);
    const Vec3 point =
        Vec3{station.leadingEdgeX, station.y, station.arcZ} +
        station.chord * u * frame.chord +
        station.chord * zeta * frame.up;
    if (!finite(point.x) || !finite(point.y) || !finite(point.z)) {
        fail(CanopyPhase::Mapping, station.id, "mapped point is non-finite");
    }
    return point;
}

std::string serializeCanopyDefinition(const CanopyDefinition& input) {
    const CanopyDefinition definition =
        validateAndNormalizeCanopyDefinition(input);
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::setprecision(std::numeric_limits<double>::max_digits10);
    output << "SOFTWING_CANOPY " << definition.schemaMajor << '\n';
    writeString(output, "IDENTIFIER", definition.identifier);
    writeString(output, "DESCRIPTION", definition.description);
    writeString(output, "UNITS_FRAME", definition.unitsFrameTag);
    output << "PROVENANCE_COUNT " << definition.provenance.size() << '\n';
    for (const CanopyProvenance& provenance : definition.provenance) {
        output << "PROVENANCE " << provenance.id.size() << ' '
               << encodeBytes(provenance.id) << ' ' << provenance.source.size()
               << ' ' << encodeBytes(provenance.source) << '\n';
    }
    output << "CELL_COUNT " << definition.cellCount << '\n';
    output << "STATION_COUNT " << definition.stations.size() << '\n';
    for (const CanopySpanStation& station : definition.stations) {
        output << "STATION " << station.id.size() << ' ' << encodeBytes(station.id)
               << ' ' << station.y << ' ' << station.leadingEdgeX << ' '
               << station.arcZ << ' ' << station.chord << ' ';
        if (definition.schemaMajor == 2) {
            output << station.sectionRollRadians << ' ';
        }
        output << station.twistRadians << ' ' << station.section.size() << '\n';
        for (const RibSectionSample& sample : station.section) {
            output << "SAMPLE " << sample.u << ' ' << sample.upper << ' '
                   << sample.lower << '\n';
        }
    }
    output << "MATERIAL_COUNT " << definition.materials.size() << '\n';
    for (const CanopyMaterial& material : definition.materials) {
        const OrthotropicMembraneMaterial& m = material.membrane;
        output << "MATERIAL " << material.id.size() << ' '
               << encodeBytes(material.id) << ' ' << material.arealDensity << ' '
               << m.warpStiffness << ' ' << m.weftStiffness << ' '
               << m.couplingStiffness << ' ' << m.shearStiffness << ' '
               << m.warpPreTension << ' ' << m.weftPreTension << ' '
               << m.dampingTime << ' ' << material.provenanceId.size() << ' '
               << encodeBytes(material.provenanceId) << '\n';
    }
    output << "ASSIGNMENT_COUNT " << definition.assignments.size() << '\n';
    for (const CanopyPanelAssignment& assignment : definition.assignments) {
        output << "ASSIGNMENT " << static_cast<std::size_t>(assignment.role) << ' '
               << assignment.materialId.size() << ' '
               << encodeBytes(assignment.materialId) << '\n';
    }
    output << "APERTURE_COUNT " << definition.apertures.size() << '\n';
    for (const CanopyApertureDefinition& aperture : definition.apertures) {
        output << "APERTURE " << aperture.id.size() << ' '
               << encodeBytes(aperture.id) << ' '
               << static_cast<std::size_t>(aperture.kind) << ' '
               << aperture.cellIndex << ' ' << aperture.ribIndex << ' '
               << aperture.firstMin << ' ' << aperture.firstMax << ' '
               << aperture.secondMin << ' ' << aperture.secondMax << ' '
               << aperture.provenanceId.size() << ' '
               << encodeBytes(aperture.provenanceId) << '\n';
    }
    output << "DIAGONAL_COUNT " << definition.diagonals.size() << '\n';
    for (const CanopyDiagonalDefinition& diagonal : definition.diagonals) {
        output << "DIAGONAL " << diagonal.id.size() << ' '
               << encodeBytes(diagonal.id) << ' ' << diagonal.cellIndex << ' '
               << diagonal.chordMin << ' ' << diagonal.chordMax << ' '
               << (diagonal.lowerOnFirstRib ? 1 : 0) << ' '
               << diagonal.materialId.size() << ' '
               << encodeBytes(diagonal.materialId) << ' '
               << diagonal.provenanceId.size() << ' '
               << encodeBytes(diagonal.provenanceId) << '\n';
    }
    output << "MESH " << definition.mesh.chordSubdivisions << ' '
           << definition.mesh.spanSubdivisionsPerCell << ' '
           << definition.mesh.thicknessSubdivisions << ' '
           << definition.mesh.coordinateTolerance << ' '
           << definition.mesh.volumeFloor << '\n';
    output << "END\n";
    return output.str();
}

CanopyDefinition parseCanopyDefinition(std::string_view text) {
    CanonicalReader reader(text);
    std::size_t version = 0;
    {
        std::istringstream header = reader.line("SOFTWING_CANOPY");
        if (!(header >> version) || (version != 1 && version != 2)) {
            fail(CanopyPhase::Parse, "header", "unsupported canopy version");
        }
        CanonicalReader::requireEnd(header, "header");
    }
    CanopyDefinition definition;
    definition.schemaMajor = version;
    definition.identifier = reader.string("IDENTIFIER");
    definition.description = reader.string("DESCRIPTION");
    definition.unitsFrameTag = reader.string("UNITS_FRAME");

    auto readEncoded = [](std::istringstream& input,
                          const std::string& entity) {
        std::size_t count = 0;
        std::string encoded;
        if (!(input >> count >> encoded)) {
            fail(CanopyPhase::Parse, entity, "invalid encoded string");
        }
        return decodeBytes(count, encoded, entity);
    };

    const std::size_t provenanceCount = reader.count("PROVENANCE_COUNT");
    definition.provenance.reserve(provenanceCount);
    for (std::size_t i = 0; i < provenanceCount; ++i) {
        std::istringstream line = reader.line("PROVENANCE");
        CanopyProvenance provenance;
        provenance.id = readEncoded(line, "provenance-id");
        provenance.source = readEncoded(line, provenance.id);
        CanonicalReader::requireEnd(line, provenance.id);
        definition.provenance.push_back(std::move(provenance));
    }
    definition.cellCount = reader.count("CELL_COUNT");
    const std::size_t stationCount = reader.count("STATION_COUNT");
    definition.stations.reserve(stationCount);
    for (std::size_t i = 0; i < stationCount; ++i) {
        std::istringstream line = reader.line("STATION");
        CanopySpanStation station;
        station.id = readEncoded(line, "station-id");
        std::size_t sampleCount = 0;
        if (!(line >> station.y >> station.leadingEdgeX >> station.arcZ >>
              station.chord)) {
            fail(CanopyPhase::Parse, station.id, "invalid station fields");
        }
        if (version == 2 && !(line >> station.sectionRollRadians)) {
            fail(CanopyPhase::Parse,
                 station.id,
                 "version 2 station is missing section roll");
        }
        if (!(line >> station.twistRadians >> sampleCount) ||
            sampleCount > kMaximumCanonicalCount) {
            fail(CanopyPhase::Parse, station.id, "invalid station fields");
        }
        reader.claimCount(sampleCount, station.id);
        CanonicalReader::requireEnd(line, station.id);
        station.section.reserve(sampleCount);
        for (std::size_t j = 0; j < sampleCount; ++j) {
            std::istringstream sampleLine = reader.line("SAMPLE");
            RibSectionSample sample;
            if (!(sampleLine >> sample.u >> sample.upper >> sample.lower)) {
                fail(CanopyPhase::Parse, station.id, "invalid section sample");
            }
            CanonicalReader::requireEnd(sampleLine, station.id);
            station.section.push_back(sample);
        }
        definition.stations.push_back(std::move(station));
    }
    const std::size_t materialCount = reader.count("MATERIAL_COUNT");
    definition.materials.reserve(materialCount);
    for (std::size_t i = 0; i < materialCount; ++i) {
        std::istringstream line = reader.line("MATERIAL");
        CanopyMaterial material;
        material.id = readEncoded(line, "material-id");
        OrthotropicMembraneMaterial& m = material.membrane;
        if (!(line >> material.arealDensity >> m.warpStiffness >>
              m.weftStiffness >> m.couplingStiffness >> m.shearStiffness >>
              m.warpPreTension >> m.weftPreTension >> m.dampingTime)) {
            fail(CanopyPhase::Parse, material.id, "invalid material fields");
        }
        material.provenanceId = readEncoded(line, material.id);
        CanonicalReader::requireEnd(line, material.id);
        definition.materials.push_back(std::move(material));
    }
    const std::size_t assignmentCount = reader.count("ASSIGNMENT_COUNT");
    definition.assignments.reserve(assignmentCount);
    for (std::size_t i = 0; i < assignmentCount; ++i) {
        std::istringstream line = reader.line("ASSIGNMENT");
        std::size_t role = 0;
        if (!(line >> role)) {
            fail(CanopyPhase::Parse, "assignment", "invalid role");
        }
        CanopyPanelAssignment assignment;
        switch (role) {
        case 0: assignment.role = CanopyPanelRole::UpperSkin; break;
        case 1: assignment.role = CanopyPanelRole::LowerSkin; break;
        case 2: assignment.role = CanopyPanelRole::LeadingEdge; break;
        case 4: assignment.role = CanopyPanelRole::Rib; break;
        case 5: assignment.role = CanopyPanelRole::Diagonal; break;
        default:
            fail(CanopyPhase::Parse,
                 "assignment-role",
                 role == 3 ? "removed trailing-edge panel role"
                           : "enum value is out of range");
        }
        assignment.materialId = readEncoded(line, "assignment-material");
        CanonicalReader::requireEnd(line, "assignment");
        definition.assignments.push_back(std::move(assignment));
    }
    const std::size_t apertureCount = reader.count("APERTURE_COUNT");
    definition.apertures.reserve(apertureCount);
    for (std::size_t i = 0; i < apertureCount; ++i) {
        std::istringstream line = reader.line("APERTURE");
        CanopyApertureDefinition aperture;
        aperture.id = readEncoded(line, "aperture-id");
        std::size_t kind = 0;
        if (!(line >> kind >> aperture.cellIndex >> aperture.ribIndex >>
              aperture.firstMin >> aperture.firstMax >> aperture.secondMin >>
              aperture.secondMax)) {
            fail(CanopyPhase::Parse, aperture.id, "invalid aperture fields");
        }
        aperture.kind =
            checkedEnum<CanopyApertureKind>(kind, 2, aperture.id);
        aperture.provenanceId = readEncoded(line, aperture.id);
        CanonicalReader::requireEnd(line, aperture.id);
        definition.apertures.push_back(std::move(aperture));
    }
    const std::size_t diagonalCount = reader.count("DIAGONAL_COUNT");
    definition.diagonals.reserve(diagonalCount);
    for (std::size_t i = 0; i < diagonalCount; ++i) {
        std::istringstream line = reader.line("DIAGONAL");
        CanopyDiagonalDefinition diagonal;
        diagonal.id = readEncoded(line, "diagonal-id");
        int lowerFirst = 0;
        if (!(line >> diagonal.cellIndex >> diagonal.chordMin >>
              diagonal.chordMax >> lowerFirst) ||
            (lowerFirst != 0 && lowerFirst != 1)) {
            fail(CanopyPhase::Parse, diagonal.id, "invalid diagonal fields");
        }
        diagonal.lowerOnFirstRib = lowerFirst != 0;
        diagonal.materialId = readEncoded(line, diagonal.id);
        diagonal.provenanceId = readEncoded(line, diagonal.id);
        CanonicalReader::requireEnd(line, diagonal.id);
        definition.diagonals.push_back(std::move(diagonal));
    }
    {
        std::istringstream line = reader.line("MESH");
        if (!(line >> definition.mesh.chordSubdivisions >>
              definition.mesh.spanSubdivisionsPerCell >>
              definition.mesh.thicknessSubdivisions >>
              definition.mesh.coordinateTolerance >> definition.mesh.volumeFloor)) {
            fail(CanopyPhase::Parse, "mesh", "invalid mesh fields");
        }
        CanonicalReader::requireEnd(line, "mesh");
    }
    reader.end();
    try {
        return validateAndNormalizeCanopyDefinition(definition);
    } catch (const CanopyError& error) {
        if (error.phase() == CanopyPhase::Validation) {
            fail(CanopyPhase::Parse, error.entity(), error.what());
        }
        throw;
    }
}

CanopyDefinition makeStraightThreeCellDefinition() {
    CanopyDefinition definition;
    definition.identifier = "straight-three-cell";
    definition.description =
        "Straight symmetric three-cell synthetic calibration canopy";
    definition.unitsFrameTag = std::string(kFrameTag);
    definition.provenance = {
        {"synthetic-geometry", "OpenSpec Stage 4 prescribed analytic fixture"},
        {"synthetic-material", "OpenSpec Stage 2 verified synthetic membrane"},
    };
    definition.cellCount = 3;
    for (std::size_t i = 0; i < 4; ++i) {
        CanopySpanStation station;
        station.id = "station-" + std::to_string(i);
        station.y = -1.5 + static_cast<double>(i);
        station.leadingEdgeX = 0.0;
        station.arcZ = 0.0;
        station.chord = 1.0;
        station.twistRadians = 0.0;
        station.section = {{0.0, 0.15, -0.15},
                           {0.25, 0.15, -0.15},
                           {0.5, 0.15, -0.15},
                           {0.75, 0.15, -0.15},
                           {1.0, 0.0, 0.0}};
        definition.stations.push_back(std::move(station));
    }
    CanopyMaterial material;
    material.id = "synthetic-fabric";
    material.arealDensity = kReferenceDensity;
    material.membrane = {800.0, 500.0, 100.0, 180.0, 0.0, 0.0, 0.02};
    material.provenanceId = "synthetic-material";
    definition.materials.push_back(material);
    for (const CanopyPanelRole role :
         {CanopyPanelRole::UpperSkin,
          CanopyPanelRole::LowerSkin,
          CanopyPanelRole::LeadingEdge,
          CanopyPanelRole::Rib,
          CanopyPanelRole::Diagonal})
        definition.assignments.push_back({role, material.id});
    for (std::size_t cell = 0; cell < 3; ++cell) {
        definition.apertures.push_back(
            {"inlet-" + std::to_string(cell),
             CanopyApertureKind::Inlet,
             cell,
             0,
             0.25,
             0.75,
             0.25,
             0.75,
             "synthetic-geometry"});
    }
    for (std::size_t rib = 1; rib < 3; ++rib) {
        definition.apertures.push_back(
            {"cross-port-" + std::to_string(rib),
             CanopyApertureKind::CrossPort,
             rib - 1,
             rib,
             0.25,
             0.75,
             0.25,
             0.75,
             "synthetic-geometry"});
    }
    definition.diagonals = {
        {"diagonal-left", 0, 0.25, 0.75, true, material.id,
         "synthetic-geometry"},
        {"diagonal-right", 2, 0.25, 0.75, false, material.id,
         "synthetic-geometry"},
    };
    definition.mesh = {3, 3, 3, 1.0e-12, 1.0e-12};
    return validateAndNormalizeCanopyDefinition(definition);
}

namespace {

struct PlannedNode {
    Vec3 position;
    double mass = 0.0;
};

struct PlannedFace {
    std::array<std::size_t, 3> nodes{};
    std::size_t panel = 0;
    std::string semanticId;
    std::array<int, 2> adjacentCells{-1, -1};
    std::string materialId;
    MaterialRole materialRole = MaterialRole::Bulk;
    std::array<Vec2, 3> chart{};
    bool virtualClosure = false;
    std::string apertureId;
    std::string provenanceId;
};

struct BuildPlan {
    explicit BuildPlan(const CanopyDefinition& normalized)
        : definition(normalized) {}

    const CanopyDefinition& definition;
    std::vector<PlannedNode> nodes;
    std::map<std::string, std::size_t> nodeByKey;
    std::vector<CanopyPanelRecord> panels;
    std::vector<PlannedFace> structural;
    std::vector<PlannedFace> virtualFaces;
    std::vector<CanopyApertureRecord> apertures;
    std::vector<CanopySeamRecord> seams;
    std::vector<CanopyMaterialBoundaryRecord> materialBoundaries;
};

std::string doubleKey(double value) {
    const std::uint64_t bits = std::bit_cast<std::uint64_t>(value);
    std::ostringstream output;
    output << std::hex << std::setw(16) << std::setfill('0') << bits;
    return output.str();
}

std::vector<double> mergedCoordinates(std::size_t subdivisions,
                                      std::span<const double> inserted,
                                      double tolerance,
                                      const std::string& entity) {
    std::vector<double> values;
    values.reserve(subdivisions + 1 + inserted.size());
    for (std::size_t i = 0; i <= subdivisions; ++i) {
        values.push_back(static_cast<double>(i) /
                         static_cast<double>(subdivisions));
    }
    values.insert(values.end(), inserted.begin(), inserted.end());
    std::ranges::sort(values);
    std::vector<double> merged;
    for (const double value : values) {
        if (!finite(value) || value < 0.0 || value > 1.0) {
            fail(CanopyPhase::Topology, entity, "grid coordinate is invalid");
        }
        if (merged.empty() || std::abs(value - merged.back()) > tolerance) {
            merged.push_back(value);
        } else if (value != merged.back()) {
            // A near-but-not-identical requested boundary is ambiguous rather than repaired.
            fail(CanopyPhase::Topology,
                 entity,
                 "grid coordinates are closer than the declared tolerance");
        }
    }
    if (merged.front() != 0.0 || merged.back() != 1.0) {
        fail(CanopyPhase::Topology, entity, "grid does not retain exact bounds");
    }
    return merged;
}

std::size_t internNode(BuildPlan& plan,
                       const std::string& key,
                       const Vec3& position) {
    const auto found = plan.nodeByKey.find(key);
    if (found != plan.nodeByKey.end()) {
        const Vec3 difference = plan.nodes[found->second].position - position;
        if (length(difference) > plan.definition.mesh.coordinateTolerance) {
            fail(CanopyPhase::Topology,
                 key,
                 "welded semantic location has inconsistent coordinates");
        }
        return found->second;
    }
    if (!finite(position.x) || !finite(position.y) || !finite(position.z)) {
        fail(CanopyPhase::Mapping, key, "planned node is non-finite");
    }
    const std::size_t index = plan.nodes.size();
    plan.nodeByKey.emplace(key, index);
    plan.nodes.push_back({position, 0.0});
    return index;
}

Vec3 stationSurfacePoint(const CanopyDefinition& definition,
                         std::size_t stationIndex,
                         double u,
                         bool upper) {
    const CanopySpanStation& station = definition.stations[stationIndex];
    return mapCanopyStationPoint(
        station, u, sectionOrdinate(station, u, upper));
}

Vec3 stationThicknessPoint(const CanopyDefinition& definition,
                           std::size_t stationIndex,
                           double u,
                           double thicknessFraction) {
    const CanopySpanStation& station = definition.stations[stationIndex];
    const double lower = sectionOrdinate(station, u, false);
    const double upper = sectionOrdinate(station, u, true);
    const double zeta = lower + thicknessFraction * (upper - lower);
    if (!(upper > lower)) {
        fail(CanopyPhase::Mapping, station.id, "generated section gap is not positive");
    }
    return mapCanopyStationPoint(station, u, zeta);
}

std::size_t stationSurfaceNode(BuildPlan& plan,
                               std::size_t station,
                               double u,
                               bool upper) {
    const std::string key = "station-surface:" + std::to_string(station) + ":" +
                            (u == 1.0 ? "trailing:" :
                             upper ? "upper:" : "lower:") + doubleKey(u);
    return internNode(plan,
                      key,
                      stationSurfacePoint(plan.definition, station, u, upper));
}

std::size_t ribNode(BuildPlan& plan,
                    std::size_t station,
                    double u,
                    double thickness) {
    if (u == 1.0)
        return stationSurfaceNode(plan, station, u, false);
    if (thickness == 0.0) {
        return stationSurfaceNode(plan, station, u, false);
    }
    if (thickness == 1.0) {
        return stationSurfaceNode(plan, station, u, true);
    }
    const std::string key = "rib:" + std::to_string(station) + ":" +
                            doubleKey(u) + ":" + doubleKey(thickness);
    return internNode(plan,
                      key,
                      stationThicknessPoint(
                          plan.definition, station, u, thickness));
}

std::size_t skinNode(BuildPlan& plan,
                     std::size_t cell,
                     double spanFraction,
                     double u,
                     bool upper) {
    if (spanFraction == 0.0) {
        return stationSurfaceNode(plan, cell, u, upper);
    }
    if (spanFraction == 1.0) {
        return stationSurfaceNode(plan, cell + 1, u, upper);
    }
    const Vec3 first =
        stationSurfacePoint(plan.definition, cell, u, upper);
    const Vec3 second =
        stationSurfacePoint(plan.definition, cell + 1, u, upper);
    const std::string key = "skin:" + std::to_string(cell) + ":" +
                            (u == 1.0 ? "trailing:" :
                             upper ? "upper:" : "lower:") +
                            doubleKey(spanFraction) + ":" + doubleKey(u);
    return internNode(plan,
                      key,
                      first + spanFraction * (second - first));
}

std::size_t edgeNode(BuildPlan& plan,
                     std::size_t cell,
                     double spanFraction,
                     double u,
                     double thickness) {
    if (spanFraction == 0.0) {
        return ribNode(plan, cell, u, thickness);
    }
    if (spanFraction == 1.0) {
        return ribNode(plan, cell + 1, u, thickness);
    }
    if (thickness == 0.0) {
        return skinNode(plan, cell, spanFraction, u, false);
    }
    if (thickness == 1.0) {
        return skinNode(plan, cell, spanFraction, u, true);
    }
    const Vec3 first =
        stationThicknessPoint(plan.definition, cell, u, thickness);
    const Vec3 second =
        stationThicknessPoint(plan.definition, cell + 1, u, thickness);
    const std::string key = "edge:" + std::to_string(cell) + ":" +
                            doubleKey(u) + ":" + doubleKey(spanFraction) +
                            ":" + doubleKey(thickness);
    return internNode(plan,
                      key,
                      first + spanFraction * (second - first));
}

std::size_t addPanel(BuildPlan& plan,
                     std::string id,
                     CanopyPanelRole role,
                     std::optional<std::size_t> cell,
                     std::optional<std::size_t> rib,
                     const std::string& materialId,
                     const std::string& provenanceId) {
    const std::size_t index = plan.panels.size();
    plan.panels.push_back({std::move(id),
                           role,
                           cell,
                           rib,
                           materialId,
                           provenanceId,
                           {}});
    return index;
}

std::array<Vec2, 3> positiveChart(const std::array<Vec2, 3>& input) {
    std::array<Vec2, 3> result = input;
    if (cross(result[1] - result[0], result[2] - result[0]) <= 0.0) {
        for (Vec2& value : result) {
            value.y = -value.y;
        }
    }
    const double determinant =
        cross(result[1] - result[0], result[2] - result[0]);
    if (!(determinant > 1.0e-14)) {
        fail(CanopyPhase::Material, "chart", "non-positive material chart");
    }
    return result;
}

void addPlannedTriangle(BuildPlan& plan,
                        std::vector<PlannedFace>& destination,
                        std::size_t panel,
                        std::array<std::size_t, 3> nodes,
                        std::array<Vec2, 3> chart,
                        const std::array<int, 2>& adjacentCells,
                        MaterialRole materialRole,
                        bool virtualClosure,
                        const std::string& apertureId,
                        const std::string& provenanceId,
                        const std::string& semanticId) {
    const Vec3& a = plan.nodes[nodes[0]].position;
    const Vec3& b = plan.nodes[nodes[1]].position;
    const Vec3& c = plan.nodes[nodes[2]].position;
    const double area2 = length(cross(b - a, c - a));
    const double scale2 = std::max({lengthSquared(b - a),
                                    lengthSquared(c - a),
                                    lengthSquared(c - b)});
    if (!finite(area2) || !(area2 > 64.0 *
                                       std::numeric_limits<double>::epsilon() *
                                       scale2)) {
        fail(CanopyPhase::Topology, semanticId, "degenerate panel triangle");
    }
    PlannedFace face;
    face.nodes = nodes;
    face.panel = panel;
    face.semanticId = semanticId;
    face.adjacentCells = adjacentCells;
    face.materialId = virtualClosure ? std::string{} :
        plan.panels[panel].materialId;
    face.materialRole = materialRole;
    face.chart = virtualClosure ? std::array<Vec2, 3>{} : positiveChart(chart);
    face.virtualClosure = virtualClosure;
    face.apertureId = apertureId;
    face.provenanceId = provenanceId;
    destination.push_back(std::move(face));
}

void emitQuad(BuildPlan& plan,
              std::size_t panel,
              const std::array<std::size_t, 4>& nodes,
              const std::array<Vec2, 4>& chart,
              const std::array<int, 2>& adjacentCells,
              MaterialRole materialRole,
              bool reverseWinding,
              bool reverseDiagonal,
              std::size_t& semanticCounter) {
    std::array<std::array<int, 3>, 2> corners{};
    if (!reverseDiagonal) {
        corners = {{{0, 1, 2}, {0, 2, 3}}};
    } else {
        corners = {{{0, 1, 3}, {1, 2, 3}}};
    }
    Vec3 firstNormal;
    double firstNormalLength = 0.0;
    for (std::size_t triangleIndex = 0; triangleIndex < corners.size();
         ++triangleIndex) {
        std::array<int, 3> triangle = corners[triangleIndex];
        if (reverseWinding) {
            std::swap(triangle[1], triangle[2]);
        }
        const Vec3& a = plan.nodes[nodes[triangle[0]]].position;
        const Vec3& b = plan.nodes[nodes[triangle[1]]].position;
        const Vec3& c = plan.nodes[nodes[triangle[2]]].position;
        const Vec3 normal = cross(b - a, c - a);
        const double normalLength = length(normal);
        if (triangleIndex == 0) {
            firstNormal = normal;
            firstNormalLength = normalLength;
        } else if (!(dot(firstNormal, normal) >
                     64.0 * std::numeric_limits<double>::epsilon() *
                         firstNormalLength * normalLength)) {
            fail(CanopyPhase::Topology,
                 plan.panels[panel].id,
                 "folded or inverted panel quad Jacobian");
        }
    }
    for (std::array<int, 3> triangle : corners) {
        if (reverseWinding) {
            std::swap(triangle[1], triangle[2]);
        }
        const std::array<std::size_t, 3> triangleNodes{
            nodes[triangle[0]], nodes[triangle[1]], nodes[triangle[2]]};
        const std::array<Vec2, 3> triangleChart{
            chart[triangle[0]], chart[triangle[1]], chart[triangle[2]]};
        addPlannedTriangle(
            plan,
            plan.structural,
            panel,
            triangleNodes,
            triangleChart,
            adjacentCells,
            materialRole,
            false,
            {},
            plan.panels[panel].provenanceId,
            plan.panels[panel].id + ":face-" +
                std::to_string(semanticCounter++));
    }
}

bool quadInside(double first0,
                double first1,
                double second0,
                double second1,
                const CanopyApertureDefinition& aperture) {
    return first0 >= aperture.firstMin && first1 <= aperture.firstMax &&
           second0 >= aperture.secondMin && second1 <= aperture.secondMax;
}

bool apertureMatchesPanel(const CanopyApertureDefinition& aperture,
                          CanopyApertureKind kind,
                          std::size_t index) {
    return aperture.kind == kind &&
           (kind == CanopyApertureKind::Inlet
                ? aperture.cellIndex == index
                : aperture.ribIndex == index);
}

bool quadInsideDeclaredAperture(const CanopyDefinition& definition,
                                CanopyApertureKind kind,
                                std::size_t index,
                                double first0,
                                double first1,
                                double second0,
                                double second1) {
    return std::ranges::any_of(
        definition.apertures,
        [&](const CanopyApertureDefinition& aperture) {
            return apertureMatchesPanel(aperture, kind, index) &&
                   quadInside(first0,
                              first1,
                              second0,
                              second1,
                              aperture);
        });
}

std::vector<double> thicknessCoordinates(const CanopyDefinition& definition);

std::vector<double> chordCoordinates(const CanopyDefinition& definition) {
    std::vector<double> inserted;
    std::vector<double> protectedBounds;
    for (const CanopyApertureDefinition& aperture : definition.apertures) {
        if (aperture.kind == CanopyApertureKind::CrossPort) {
            inserted.push_back(aperture.firstMin);
            inserted.push_back(aperture.firstMax);
            protectedBounds.push_back(aperture.firstMin);
            protectedBounds.push_back(aperture.firstMax);
        }
    }
    for (const CanopyDiagonalDefinition& diagonal : definition.diagonals) {
        inserted.push_back(diagonal.chordMin);
        inserted.push_back(diagonal.chordMax);
        protectedBounds.push_back(diagonal.chordMin);
        protectedBounds.push_back(diagonal.chordMax);
    }
    for (const CanopySpanStation& station : definition.stations) {
        for (const RibSectionSample& sample : station.section) {
            inserted.push_back(sample.u);
        }
    }
    std::vector<double> values = mergedCoordinates(
        definition.mesh.chordSubdivisions,
        inserted,
        definition.mesh.coordinateTolerance,
        "chord-grid");
    std::erase_if(values, [&](double value) {
        if (std::ranges::find(protectedBounds, value) !=
            protectedBounds.end()) {
            return false;
        }
        return std::ranges::any_of(
            definition.apertures,
            [&](const CanopyApertureDefinition& aperture) {
                return aperture.kind == CanopyApertureKind::CrossPort &&
                       value > aperture.firstMin && value < aperture.firstMax;
            });
    });
    // Bound the chord-wise aspect ratio of every rib strip. The rib grid
    // pairs this chord list with thickness rows that can be a small fraction
    // of the LOCAL section gap (inlet lips, cross-port bounds), so a chord
    // interval sized for the skins can span a strip whose height collapses
    // near the nose and the sharp trailing edge. Such sliver strips have
    // near-degenerate rest charts: the membrane element solve loses
    // conditioning, the strip goes numerically limp, and the pressurized
    // nose tears open (measured on the studio wing as ~0.5 MJ of fictitious
    // elastic energy at rib noses against ~230 J in the skins, and
    // SymmetricMatrix3 singularity at flight timesteps). Subdividing the
    // chord until each interval is at most kMaximumRibStripAspect times the
    // smallest adjacent strip height keeps every rest chart conditioned for
    // any wing composition; the bound sits well under the ~1:50 aspect that
    // measurably fails and above the ~1:20 the archived healthy canopies
    // carried. Midpoints inside a cross-port cut, or closer to an existing
    // line than the declared tolerance, are not inserted.
    constexpr double kMaximumRibStripAspect = 16.0;
    constexpr int kMaximumAspectSplitDepth = 3;
    const std::vector<double> rows = thicknessCoordinates(definition);
    double minimumRowFraction = 1.0;
    for (std::size_t j = 0; j + 1 < rows.size(); ++j)
        minimumRowFraction =
            std::min(minimumRowFraction, rows[j + 1] - rows[j]);
    const auto minimumStripHeight = [&](double u0, double u1) {
        double height = std::numeric_limits<double>::infinity();
        for (const CanopySpanStation& station : definition.stations) {
            const double gap0 = sectionOrdinate(station, u0, true) -
                                sectionOrdinate(station, u0, false);
            const double gap1 = sectionOrdinate(station, u1, true) -
                                sectionOrdinate(station, u1, false);
            height = std::min(height,
                              station.chord * std::max(gap0, gap1) *
                                  minimumRowFraction);
        }
        return height;
    };
    const auto insideCrossPort = [&](double value) {
        return std::ranges::any_of(
            definition.apertures,
            [&](const CanopyApertureDefinition& aperture) {
                return aperture.kind == CanopyApertureKind::CrossPort &&
                       value > aperture.firstMin && value < aperture.firstMax;
            });
    };
    std::vector<double> refined;
    refined.reserve(values.size());
    const auto refine = [&](auto&& self, double u0, double u1,
                            int depth) -> void {
        const double height = minimumStripHeight(u0, u1);
        const double midpoint = 0.5 * (u0 + u1);
        if (depth < kMaximumAspectSplitDepth &&
            u1 - u0 > kMaximumRibStripAspect * height &&
            !insideCrossPort(midpoint) &&
            midpoint - u0 > definition.mesh.coordinateTolerance) {
            self(self, u0, midpoint, depth + 1);
            self(self, midpoint, u1, depth + 1);
            return;
        }
        refined.push_back(u1);
    };
    refined.push_back(values.front());
    for (std::size_t i = 0; i + 1 < values.size(); ++i)
        refine(refine, values[i], values[i + 1], 0);
    return refined;
}

std::vector<double> thicknessCoordinates(
    const CanopyDefinition& definition) {
    std::vector<double> inserted;
    for (const CanopyApertureDefinition& aperture : definition.apertures) {
        inserted.push_back(aperture.secondMin);
        inserted.push_back(aperture.secondMax);
    }
    std::vector<double> values = mergedCoordinates(
        definition.mesh.thicknessSubdivisions,
        inserted,
        definition.mesh.coordinateTolerance,
        "thickness-grid");
    std::erase_if(values, [&](double value) {
        if (std::ranges::find(inserted, value) != inserted.end()) {
            return false;
        }
        return std::ranges::any_of(
            definition.apertures,
            [&](const CanopyApertureDefinition& aperture) {
                return value > aperture.secondMin &&
                       value < aperture.secondMax;
            });
    });
    return values;
}

std::vector<double> spanCoordinates(const CanopyDefinition& definition,
                                    std::size_t cell) {
    std::vector<double> inserted;
    for (const CanopyApertureDefinition& aperture : definition.apertures) {
        if (apertureMatchesPanel(
                aperture, CanopyApertureKind::Inlet, cell)) {
            inserted.push_back(aperture.firstMin);
            inserted.push_back(aperture.firstMax);
        }
    }
    std::vector<double> values = mergedCoordinates(
        definition.mesh.spanSubdivisionsPerCell,
        inserted,
        definition.mesh.coordinateTolerance,
        "span-grid-cell-" + std::to_string(cell));
    std::erase_if(values, [&](double value) {
        if (std::ranges::find(inserted, value) != inserted.end()) {
            return false;
        }
        return std::ranges::any_of(
            definition.apertures,
            [&](const CanopyApertureDefinition& aperture) {
                return apertureMatchesPanel(
                           aperture, CanopyApertureKind::Inlet, cell) &&
                       value > aperture.firstMin && value < aperture.firstMax;
            });
    });
    return values;
}

double triangleArea(const std::vector<PlannedNode>& nodes,
                    const PlannedFace& face) {
    const Vec3& a = nodes[face.nodes[0]].position;
    const Vec3& b = nodes[face.nodes[1]].position;
    const Vec3& c = nodes[face.nodes[2]].position;
    return 0.5 * length(cross(b - a, c - a));
}

std::size_t panelIndexById(const BuildPlan& plan, const std::string& id) {
    const auto found = std::ranges::find_if(
        plan.panels,
        [&](const CanopyPanelRecord& panel) { return panel.id == id; });
    if (found == plan.panels.end()) {
        fail(CanopyPhase::Topology, id, "unknown panel id");
    }
    return static_cast<std::size_t>(found - plan.panels.begin());
}

void addSeam(BuildPlan& plan,
             std::string id,
             const std::string& firstPanel,
             const std::string& secondPanel,
             CanopySeamKind kind,
             std::vector<std::size_t> nodes,
             const std::string& provenance) {
    if (nodes.size() < 2) {
        fail(CanopyPhase::Topology, id, "seam requires an ordered node chain");
    }
    if (firstPanel == secondPanel) {
        fail(CanopyPhase::Topology,
             id,
             "welded seam must join two distinct semantic panels");
    }
    std::set<std::size_t> unique(nodes.begin(), nodes.end());
    if (unique.size() != nodes.size()) {
        fail(CanopyPhase::Topology, id, "seam contains a duplicate node");
    }
    const Vec3 direction = normalized(plan.nodes[nodes.back()].position -
                                      plan.nodes[nodes.front()].position);
    if (!(lengthSquared(direction) > 0.0)) {
        fail(CanopyPhase::Topology, id, "seam direction is degenerate");
    }
    static_cast<void>(panelIndexById(plan, firstPanel));
    static_cast<void>(panelIndexById(plan, secondPanel));
    plan.seams.push_back({std::move(id),
                          firstPanel,
                          secondPanel,
                          kind,
                          direction,
                          std::move(nodes),
                          provenance});
}

BuildPlan makeBuildPlan(const CanopyDefinition& definition) {
    BuildPlan plan(definition);
    const std::vector<double> chord = chordCoordinates(definition);
    const std::vector<double> thickness = thicknessCoordinates(definition);
    const std::string geometryProvenance = definition.provenance.front().id;

    std::size_t semanticCounter = 0;
    for (std::size_t cell = 0; cell < definition.cellCount; ++cell) {
        const std::vector<double> span = spanCoordinates(definition, cell);
        for (const CanopyPanelRole role :
             {CanopyPanelRole::UpperSkin, CanopyPanelRole::LowerSkin}) {
            const bool upper = role == CanopyPanelRole::UpperSkin;
            const CanopyPanelAssignment& assignment = assignmentFor(definition, role);
            const std::string id = std::string(canopyPanelRoleName(role)) + "-" +
                                   std::to_string(cell);
            const std::size_t panel = addPanel(plan,
                                               id,
                                               role,
                                               cell,
                                               std::nullopt,
                                               assignment.materialId,
                                               geometryProvenance);
            const double y0 = definition.stations[cell].y;
            const double y1 = definition.stations[cell + 1].y;
            const double meanChord = 0.5 *
                (definition.stations[cell].chord +
                 definition.stations[cell + 1].chord);
            for (std::size_t j = 0; j + 1 < span.size(); ++j) {
                for (std::size_t i = 0; i + 1 < chord.size(); ++i) {
                    const double u0 = chord[i];
                    const double u1 = chord[i + 1];
                    const double v0 = span[j];
                    const double v1 = span[j + 1];
                    const std::array<std::size_t, 4> nodes{
                        skinNode(plan, cell, v0, u0, upper),
                        skinNode(plan, cell, v0, u1, upper),
                        skinNode(plan, cell, v1, u1, upper),
                        skinNode(plan, cell, v1, u0, upper),
                    };
                    const std::array<Vec2, 4> chart{
                        Vec2{meanChord * u0, y0 + v0 * (y1 - y0)},
                        Vec2{meanChord * u1, y0 + v0 * (y1 - y0)},
                        Vec2{meanChord * u1, y0 + v1 * (y1 - y0)},
                        Vec2{meanChord * u0, y0 + v1 * (y1 - y0)},
                    };
                    const bool mirroredParity =
                        ((i + j + std::min(cell,
                                           definition.cellCount - 1 - cell)) & 1U) != 0;
                    emitQuad(plan,
                             panel,
                             nodes,
                             chart,
                             {static_cast<int>(cell), -1},
                             MaterialRole::Bulk,
                             !upper,
                             mirroredParity,
                             semanticCounter);
                }
            }
        }

        for (const CanopyPanelRole role : {CanopyPanelRole::LeadingEdge}) {
            constexpr bool leading = true;
            constexpr double u = 0.0;
            const CanopyPanelAssignment& assignment = assignmentFor(definition, role);
            const std::string id = std::string(canopyPanelRoleName(role)) + "-" +
                                   std::to_string(cell);
            const std::size_t panel = addPanel(plan,
                                               id,
                                               role,
                                               cell,
                                               std::nullopt,
                                               assignment.materialId,
                                               geometryProvenance);
            const double y0 = definition.stations[cell].y;
            const double y1 = definition.stations[cell + 1].y;
            const double gap0 = definition.stations[cell].chord *
                (sectionOrdinate(definition.stations[cell], u, true) -
                 sectionOrdinate(definition.stations[cell], u, false));
            const double gap1 = definition.stations[cell + 1].chord *
                (sectionOrdinate(definition.stations[cell + 1], u, true) -
                 sectionOrdinate(definition.stations[cell + 1], u, false));
            const double meanGap = 0.5 * (gap0 + gap1);
            for (std::size_t j = 0; j + 1 < span.size(); ++j) {
                for (std::size_t i = 0; i + 1 < thickness.size(); ++i) {
                    const double v0 = span[j];
                    const double v1 = span[j + 1];
                    const double t0 = thickness[i];
                    const double t1 = thickness[i + 1];
                    if (leading && quadInsideDeclaredAperture(
                                       definition,
                                       CanopyApertureKind::Inlet,
                                       cell,
                                       v0,
                                       v1,
                                       t0,
                                       t1)) {
                        continue;
                    }
                    const std::array<std::size_t, 4> nodes{
                        edgeNode(plan, cell, v0, u, t0),
                        edgeNode(plan, cell, v1, u, t0),
                        edgeNode(plan, cell, v1, u, t1),
                        edgeNode(plan, cell, v0, u, t1),
                    };
                    const std::array<Vec2, 4> chart{
                        Vec2{y0 + v0 * (y1 - y0), meanGap * t0},
                        Vec2{y0 + v1 * (y1 - y0), meanGap * t0},
                        Vec2{y0 + v1 * (y1 - y0), meanGap * t1},
                        Vec2{y0 + v0 * (y1 - y0), meanGap * t1},
                    };
                    emitQuad(plan,
                             panel,
                             nodes,
                             chart,
                             {static_cast<int>(cell), -1},
                             MaterialRole::Bulk,
                             leading,
                             ((i + j + cell) & 1U) != 0,
                             semanticCounter);
                }
            }
        }
    }

    for (std::size_t rib = 0; rib <= definition.cellCount; ++rib) {
        const CanopyPanelAssignment& assignment =
            assignmentFor(definition, CanopyPanelRole::Rib);
        const std::string id = "rib-" + std::to_string(rib);
        const std::size_t panel = addPanel(plan,
                                           id,
                                           CanopyPanelRole::Rib,
                                           std::nullopt,
                                           rib,
                                           assignment.materialId,
                                           geometryProvenance);
        const double chordScale = definition.stations[rib].chord;
        for (std::size_t j = 0; j + 1 < thickness.size(); ++j) {
            for (std::size_t i = 0; i + 1 < chord.size(); ++i) {
                const double u0 = chord[i];
                const double u1 = chord[i + 1];
                const double t0 = thickness[j];
                const double t1 = thickness[j + 1];
                const double gap0 = chordScale *
                    (sectionOrdinate(definition.stations[rib], u0, true) -
                     sectionOrdinate(definition.stations[rib], u0, false));
                const double gap1 = chordScale *
                    (sectionOrdinate(definition.stations[rib], u1, true) -
                     sectionOrdinate(definition.stations[rib], u1, false));
                if (quadInsideDeclaredAperture(
                        definition,
                        CanopyApertureKind::CrossPort,
                        rib,
                        u0,
                        u1,
                        t0,
                        t1)) {
                    continue;
                }
                const std::array<std::size_t, 4> nodes{
                    ribNode(plan, rib, u0, t0),
                    ribNode(plan, rib, u1, t0),
                    ribNode(plan, rib, u1, t1),
                    ribNode(plan, rib, u0, t1),
                };
                const std::array<Vec2, 4> chart{
                    Vec2{chordScale * u0, gap0 * t0},
                    Vec2{chordScale * u1, gap1 * t0},
                    Vec2{chordScale * u1, gap1 * t1},
                    Vec2{chordScale * u0, gap0 * t1},
                };
                const int lowerCell = rib == 0 ? -1 : static_cast<int>(rib - 1);
                const int upperCell = rib == definition.cellCount
                                          ? -1
                                          : static_cast<int>(rib);
                if (u1 == 1.0) {
                    addPlannedTriangle(
                        plan,
                        plan.structural,
                        panel,
                        {nodes[0], nodes[3], nodes[1]},
                        {chart[0], chart[3], chart[1]},
                        {lowerCell, upperCell},
                        MaterialRole::Bulk,
                        false,
                        {},
                        plan.panels[panel].provenanceId,
                        plan.panels[panel].id + ":face-" +
                            std::to_string(semanticCounter++));
                } else {
                    emitQuad(plan,
                             panel,
                             nodes,
                             chart,
                             {lowerCell, upperCell},
                             MaterialRole::Bulk,
                             true,
                             ((i + j + std::min(
                                 rib, definition.cellCount - rib)) & 1U) != 0,
                             semanticCounter);
                }
            }
        }
    }

    for (const CanopyDiagonalDefinition& diagonal : definition.diagonals) {
        const std::string id = diagonal.id;
        const std::size_t panel = addPanel(plan,
                                           id,
                                           CanopyPanelRole::Diagonal,
                                           diagonal.cellIndex,
                                           std::nullopt,
                                           diagonal.materialId,
                                           diagonal.provenanceId);
        std::vector<double> webChord;
        for (const double u : chord) {
            if (u >= diagonal.chordMin && u <= diagonal.chordMax) {
                webChord.push_back(u);
            }
        }
        if (webChord.empty() || webChord.front() != diagonal.chordMin ||
            webChord.back() != diagonal.chordMax) {
            fail(CanopyPhase::Topology, diagonal.id, "diagonal bounds missing from grid");
        }
        const std::size_t firstRib = diagonal.cellIndex;
        const std::size_t secondRib = diagonal.cellIndex + 1;
        const double firstThickness = diagonal.lowerOnFirstRib ? 0.0 : 1.0;
        const double secondThickness = diagonal.lowerOnFirstRib ? 1.0 : 0.0;
        plan.materialBoundaries.push_back(
            {diagonal.id + "-chord-min-boundary",
             diagonal.id,
             CanopyMaterialBoundaryKind::DiagonalChordEnd,
             {ribNode(plan, firstRib, webChord.front(), firstThickness),
              ribNode(plan, secondRib, webChord.front(), secondThickness)},
             diagonal.provenanceId});
        plan.materialBoundaries.push_back(
            {diagonal.id + "-chord-max-boundary",
             diagonal.id,
             CanopyMaterialBoundaryKind::DiagonalChordEnd,
             {ribNode(plan, firstRib, webChord.back(), firstThickness),
              ribNode(plan, secondRib, webChord.back(), secondThickness)},
             diagonal.provenanceId});
        for (std::size_t i = 0; i + 1 < webChord.size(); ++i) {
            const double u0 = webChord[i];
            const double u1 = webChord[i + 1];
            const std::size_t a = ribNode(plan, firstRib, u0, firstThickness);
            const std::size_t b = ribNode(plan, firstRib, u1, firstThickness);
            const std::size_t c = ribNode(plan, secondRib, u1, secondThickness);
            const std::size_t d = ribNode(plan, secondRib, u0, secondThickness);
            const double across0 = length(plan.nodes[d].position -
                                          plan.nodes[a].position);
            const double across1 = length(plan.nodes[c].position -
                                          plan.nodes[b].position);
            const double across = 0.5 * (across0 + across1);
            emitQuad(plan,
                     panel,
                     {a, b, c, d},
                     {Vec2{u0, 0.0},
                      Vec2{u1, 0.0},
                      Vec2{u1, across},
                      Vec2{u0, across}},
                     {-1, -1},
                     MaterialRole::Reinforcement,
                     false,
                     (i & 1U) != 0,
                     semanticCounter);
        }
    }

    // Declared holes receive exactly two virtual lip-corner triangles after all material.
    for (const CanopyApertureDefinition& aperture : definition.apertures) {
        std::array<std::size_t, 4> lip{};
        std::vector<std::size_t> boundaryLoop;
        std::size_t panel = 0;
        std::array<int, 2> adjacent{-1, -1};
        bool reverse = false;
        if (aperture.kind == CanopyApertureKind::Inlet) {
            const std::size_t cell = aperture.cellIndex;
            panel = panelIndexById(plan, "leading-edge-" + std::to_string(cell));
            lip = {edgeNode(plan, cell, aperture.firstMin, 0.0, aperture.secondMin),
                   edgeNode(plan, cell, aperture.firstMax, 0.0, aperture.secondMin),
                   edgeNode(plan, cell, aperture.firstMax, 0.0, aperture.secondMax),
                   edgeNode(plan, cell, aperture.firstMin, 0.0, aperture.secondMax)};
            adjacent = {static_cast<int>(cell), -1};
            reverse = true;
            const std::vector<double> firstValues =
                spanCoordinates(definition, cell);
            auto nodeAt = [&](double first, double second) {
                return edgeNode(plan, cell, first, 0.0, second);
            };
            for (const double first : firstValues) {
                if (first >= aperture.firstMin &&
                    first <= aperture.firstMax) {
                    boundaryLoop.push_back(nodeAt(first, aperture.secondMin));
                }
            }
            for (const double second : thickness) {
                if (second > aperture.secondMin &&
                    second <= aperture.secondMax) {
                    boundaryLoop.push_back(nodeAt(aperture.firstMax, second));
                }
            }
            for (auto value = firstValues.rbegin();
                 value != firstValues.rend(); ++value) {
                if (*value >= aperture.firstMin &&
                    *value < aperture.firstMax) {
                    boundaryLoop.push_back(nodeAt(*value, aperture.secondMax));
                }
            }
            for (auto value = thickness.rbegin(); value != thickness.rend();
                 ++value) {
                if (*value > aperture.secondMin &&
                    *value < aperture.secondMax) {
                    boundaryLoop.push_back(nodeAt(aperture.firstMin, *value));
                }
            }
        } else {
            const std::size_t rib = aperture.ribIndex;
            panel = panelIndexById(plan, "rib-" + std::to_string(rib));
            lip = {ribNode(plan, rib, aperture.firstMin, aperture.secondMin),
                   ribNode(plan, rib, aperture.firstMax, aperture.secondMin),
                   ribNode(plan, rib, aperture.firstMax, aperture.secondMax),
                   ribNode(plan, rib, aperture.firstMin, aperture.secondMax)};
            adjacent = {static_cast<int>(rib - 1), static_cast<int>(rib)};
            reverse = true;
            auto nodeAt = [&](double first, double second) {
                return ribNode(plan, rib, first, second);
            };
            for (const double first : chord) {
                if (first >= aperture.firstMin &&
                    first <= aperture.firstMax) {
                    boundaryLoop.push_back(nodeAt(first, aperture.secondMin));
                }
            }
            for (const double second : thickness) {
                if (second > aperture.secondMin &&
                    second <= aperture.secondMax) {
                    boundaryLoop.push_back(nodeAt(aperture.firstMax, second));
                }
            }
            for (auto value = chord.rbegin(); value != chord.rend(); ++value) {
                if (*value >= aperture.firstMin &&
                    *value < aperture.firstMax) {
                    boundaryLoop.push_back(nodeAt(*value, aperture.secondMax));
                }
            }
            for (auto value = thickness.rbegin(); value != thickness.rend();
                 ++value) {
                if (*value > aperture.secondMin &&
                    *value < aperture.secondMax) {
                    boundaryLoop.push_back(nodeAt(aperture.firstMin, *value));
                }
            }
        }
        if (boundaryLoop.size() < 4) {
            fail(CanopyPhase::Topology,
                 aperture.id,
                 "aperture boundary loop is incomplete");
        }
        std::array<std::array<int, 3>, 2> corners{{{0, 1, 2}, {0, 2, 3}}};
        std::array<std::size_t, 2> closureIndices{};
        for (std::size_t i = 0; i < 2; ++i) {
            if (reverse) {
                std::swap(corners[i][1], corners[i][2]);
            }
            closureIndices[i] = plan.virtualFaces.size();
            addPlannedTriangle(plan,
                               plan.virtualFaces,
                               panel,
                               {lip[corners[i][0]],
                                lip[corners[i][1]],
                                lip[corners[i][2]]},
                               {},
                               adjacent,
                               MaterialRole::Bulk,
                               true,
                               aperture.id,
                               aperture.provenanceId,
                               aperture.id + ":closure-" + std::to_string(i));
        }
        const Vec3 p0 = plan.nodes[lip[0]].position;
        const Vec3 p1 = plan.nodes[lip[1]].position;
        const Vec3 p2 = plan.nodes[lip[2]].position;
        const Vec3 p3 = plan.nodes[lip[3]].position;
        const Vec3 firstAreaVector = 0.5 * cross(p1 - p0, p2 - p0);
        const Vec3 secondAreaVector = 0.5 * cross(p2 - p0, p3 - p0);
        Vec3 normal = normalized(firstAreaVector + secondAreaVector);
        std::array<std::size_t, 4> orientedLip = lip;
        if (reverse) {
            normal = -normal;
            orientedLip = {lip[0], lip[3], lip[2], lip[1]};
            std::vector<std::size_t> reversedBoundary;
            reversedBoundary.reserve(boundaryLoop.size());
            reversedBoundary.push_back(boundaryLoop.front());
            for (std::size_t i = boundaryLoop.size(); i-- > 1;) {
                reversedBoundary.push_back(boundaryLoop[i]);
            }
            boundaryLoop = std::move(reversedBoundary);
        }
        const double firstArea = length(firstAreaVector);
        const double secondArea = length(secondAreaVector);
        const double area = firstArea + secondArea;
        const Vec3 centroid =
            (firstArea * ((p0 + p1 + p2) / 3.0) +
             secondArea * ((p0 + p2 + p3) / 3.0)) /
            area;
        plan.apertures.push_back({aperture.id,
                                  aperture.kind,
                                  adjacent,
                                  area,
                                  centroid,
                                  normal,
                                  orientedLip,
                                  boundaryLoop,
                                  closureIndices,
                                  aperture.provenanceId,
                                  CanopyValueProvenance::Derived});
    }

    // Welded seam chains use the exact shared node identities already interned above.
    for (std::size_t rib = 0; rib <= definition.cellCount; ++rib) {
        std::vector<std::size_t> adjacentCells;
        if (rib > 0) {
            adjacentCells.push_back(rib - 1);
        }
        if (rib < definition.cellCount) {
            adjacentCells.push_back(rib);
        }
        for (const bool upper : {false, true}) {
            std::vector<std::size_t> nodes;
            for (const double u : chord) {
                nodes.push_back(stationSurfaceNode(plan, rib, u, upper));
            }
            for (const std::size_t adjacentCell : adjacentCells) {
                addSeam(plan,
                        "seam-rib-" + std::to_string(rib) +
                            (upper ? "-upper-cell-" : "-lower-cell-") +
                            std::to_string(adjacentCell),
                        (upper ? "upper-skin-" : "lower-skin-") +
                            std::to_string(adjacentCell),
                        "rib-" + std::to_string(rib),
                        CanopySeamKind::SkinRib,
                        nodes,
                        geometryProvenance);
            }
        }
        std::vector<std::size_t> leadingNodes;
        for (const double t : thickness)
            leadingNodes.push_back(ribNode(plan, rib, 0.0, t));
        for (const std::size_t adjacentCell : adjacentCells) {
            addSeam(plan,
                    "seam-rib-" + std::to_string(rib) +
                        "-leading-cell-" + std::to_string(adjacentCell),
                    "rib-" + std::to_string(rib),
                    "leading-edge-" + std::to_string(adjacentCell),
                    CanopySeamKind::RibEdge,
                    leadingNodes,
                    geometryProvenance);
        }
    }
    for (std::size_t cell = 0; cell < definition.cellCount; ++cell) {
        const std::vector<double> span = spanCoordinates(definition, cell);
        for (const bool upper : {false, true}) {
            std::vector<std::size_t> nodes;
            for (const double v : span) {
                nodes.push_back(edgeNode(plan,
                                         cell,
                                         v,
                                         0.0,
                                         upper ? 1.0 : 0.0));
            }
            addSeam(plan,
                    "seam-cell-" + std::to_string(cell) +
                        (upper ? "-upper-leading" : "-lower-leading"),
                    (upper ? "upper-skin-" : "lower-skin-") +
                        std::to_string(cell),
                    "leading-edge-" + std::to_string(cell),
                    CanopySeamKind::SkinEdge,
                    std::move(nodes),
                    geometryProvenance);
        }
        std::vector<std::size_t> trailingNodes;
        for (const double v : span)
            trailingNodes.push_back(skinNode(plan, cell, v, 1.0, false));
        addSeam(plan,
                "seam-cell-" + std::to_string(cell) + "-sharp-trailing",
                "upper-skin-" + std::to_string(cell),
                "lower-skin-" + std::to_string(cell),
                CanopySeamKind::SkinEdge,
                std::move(trailingNodes),
                geometryProvenance);
    }
    for (const CanopyDiagonalDefinition& diagonal : definition.diagonals) {
        std::vector<std::size_t> firstNodes;
        std::vector<std::size_t> secondNodes;
        for (const double u : chord) {
            if (u >= diagonal.chordMin && u <= diagonal.chordMax) {
                firstNodes.push_back(ribNode(plan,
                                             diagonal.cellIndex,
                                             u,
                                             diagonal.lowerOnFirstRib ? 0.0 : 1.0));
                secondNodes.push_back(ribNode(plan,
                                              diagonal.cellIndex + 1,
                                              u,
                                              diagonal.lowerOnFirstRib ? 1.0 : 0.0));
            }
        }
        addSeam(plan,
                diagonal.id + "-first-attachment",
                diagonal.id,
                "rib-" + std::to_string(diagonal.cellIndex),
                CanopySeamKind::DiagonalAttachment,
                std::move(firstNodes),
                diagonal.provenanceId);
        addSeam(plan,
                diagonal.id + "-second-attachment",
                diagonal.id,
                "rib-" + std::to_string(diagonal.cellIndex + 1),
                CanopySeamKind::DiagonalAttachment,
                std::move(secondNodes),
                diagonal.provenanceId);
    }
    return plan;
}

double signedVolume(std::span<const Node> nodes,
                    std::span<const Triangle> triangles) {
    double volume = 0.0;
    for (const Triangle& triangle : triangles) {
        const Vec3& a = nodes[triangle.a].position;
        const Vec3& b = nodes[triangle.b].position;
        const Vec3& c = nodes[triangle.c].position;
        volume += dot(a, cross(b, c)) / 6.0;
    }
    return volume;
}

Vec3 volumeCentroid(std::span<const Node> nodes,
                    std::span<const Triangle> triangles,
                    double volume) {
    Vec3 weighted;
    for (const Triangle& triangle : triangles) {
        const Vec3& a = nodes[triangle.a].position;
        const Vec3& b = nodes[triangle.b].position;
        const Vec3& c = nodes[triangle.c].position;
        const double tetraVolume = dot(a, cross(b, c)) / 6.0;
        weighted += 0.25 * tetraVolume * (a + b + c);
    }
    return weighted / volume;
}

std::pair<Vec3, Vec3> closureResiduals(std::span<const Node> nodes,
                                      std::span<const Triangle> triangles,
                                      const Vec3& momentOrigin) {
    Vec3 force;
    Vec3 moment;
    for (const Triangle& triangle : triangles) {
        const Vec3& a = nodes[triangle.a].position;
        const Vec3& b = nodes[triangle.b].position;
        const Vec3& c = nodes[triangle.c].position;
        const Vec3 areaVector = 0.5 * cross(b - a, c - a);
        const Vec3 centroid = (a + b + c) / 3.0;
        force += areaVector;
        moment += cross(centroid - momentOrigin, areaVector);
    }
    return {force, moment};
}

} // namespace

CanopyMesh buildCanopy(const CanopyDefinition& input) {
    const CanopyDefinition definition =
        validateAndNormalizeCanopyDefinition(input);
    BuildPlan plan = makeBuildPlan(definition);

    std::array<double, canopyPanelRoleCount> areaByRole{};
    std::array<double, 3> areaByMaterialRole{};
    double materialArea = 0.0;
    double materialMass = 0.0;
    std::vector<double> faceMass(plan.structural.size(), 0.0);
    for (std::size_t faceIndex = 0; faceIndex < plan.structural.size();
         ++faceIndex) {
        const PlannedFace& face = plan.structural[faceIndex];
        const double area = triangleArea(plan.nodes, face);
        const CanopyMaterial& material = materialById(definition, face.materialId);
        const double mass = area * material.arealDensity;
        if (!(area > 0.0) || !finite(area) || !(mass > 0.0) || !finite(mass)) {
            fail(CanopyPhase::Material,
                 face.semanticId,
                 "invalid triangle area or material mass");
        }
        faceMass[faceIndex] = mass;
        materialArea += area;
        materialMass += mass;
        areaByRole[canopyPanelRoleIndex(plan.panels[face.panel].role)] += area;
        areaByMaterialRole[static_cast<std::size_t>(face.materialRole)] += area;
        for (const std::size_t node : face.nodes) {
            plan.nodes[node].mass += mass / 3.0;
        }
    }
    for (std::size_t i = 0; i < plan.nodes.size(); ++i) {
        if (!(plan.nodes[i].mass > 0.0) || !finite(plan.nodes[i].mass)) {
            fail(CanopyPhase::Material,
                 "node-" + std::to_string(i),
                 "material vertex has no positive incident mass");
        }
    }

    CanopyMesh result;
    result.definition = definition;
    result.panels = plan.panels;
    result.apertures = plan.apertures;
    result.seams = plan.seams;
    result.materialBoundaries = plan.materialBoundaries;
    result.materialArea = materialArea;
    result.materialMass = materialMass;
    result.areaByRole = areaByRole;
    result.areaByMaterialRole = areaByMaterialRole;

    Vec3 weightedCentre;
    for (const PlannedNode& node : plan.nodes) {
        result.body.addNode(node.position, node.mass);
        weightedCentre += node.mass * node.position;
    }
    result.centreOfMass = weightedCentre / materialMass;

    std::vector<MembraneElementDefinition> membraneDefinitions;
    membraneDefinitions.reserve(plan.structural.size());
    const auto neutralMembraneChart = [&](const PlannedFace& face) {
        if (definition.schemaMajor != 2) return face.chart;
        const Vec3& p0 = result.body.nodes()[face.nodes[0]].position;
        const Vec3& p1 = result.body.nodes()[face.nodes[1]].position;
        const Vec3& p2 = result.body.nodes()[face.nodes[2]].position;
        const Vec3 first = p1 - p0;
        const Vec3 second = p2 - p0;
        const double firstLength = length(first);
        const Vec3 localX = first / firstLength;
        const Vec3 secondNormal = second - dot(second, localX) * localX;
        const Vec3 localY = normalized(secondNormal);

        const Vec2 sourceFirst = face.chart[1] - face.chart[0];
        const Vec2 sourceSecond = face.chart[2] - face.chart[0];
        const Matrix2 sourceMatrix{sourceFirst.x, sourceSecond.x,
                                   sourceFirst.y, sourceSecond.y};
        const Matrix2 inverseSource = checkedInverse(sourceMatrix);
        Vec3 warp = first * inverseSource.m00 +
                    second * inverseSource.m10;
        warp = normalized(warp);
        Vec2 warpInLocal{dot(warp, localX), dot(warp, localY)};
        warpInLocal *= 1.0 / std::sqrt(lengthSquared(warpInLocal));
        const auto materialCoordinates = [&](const Vec2& local) {
            return Vec2{dot(local, warpInLocal),
                        cross(warpInLocal, local)};
        };
        return std::array<Vec2, 3>{
            Vec2{},
            materialCoordinates({firstLength, 0.0}),
            materialCoordinates({dot(second, localX),
                                 dot(second, localY)})};
    };
    auto publishFace = [&](const PlannedFace& face, bool structural) {
        const std::size_t triangle = result.body.addTriangle(
            face.nodes[0], face.nodes[1], face.nodes[2]);
        CanopyFaceRecord record;
        record.triangle = triangle;
        record.semanticId = face.semanticId;
        record.panelId = face.virtualClosure
                             ? std::string{}
                             : result.panels[face.panel].id;
        record.panelRole = result.panels[face.panel].role;
        record.adjacentCells = face.adjacentCells;
        record.materialId = face.materialId;
        record.materialRole = face.materialRole;
        record.chart = face.chart;
        record.virtualClosure = face.virtualClosure;
        record.apertureId = face.apertureId;
        record.provenanceId = face.provenanceId;
        result.faces.push_back(std::move(record));
        if (structural) {
            result.structuralTriangles.push_back(triangle);
            result.fabricContactTriangles.push_back(triangle);
            result.panels[face.panel].triangles.push_back(triangle);
            membraneDefinitions.push_back({triangle,
                                           neutralMembraneChart(face),
                                           materialById(definition,
                                                        face.materialId)
                                               .membrane,
                                           face.materialRole});
        } else {
            result.virtualTriangles.push_back(triangle);
        }
    };
    for (const PlannedFace& face : plan.structural) {
        publishFace(face, true);
    }
    const std::size_t structuralCount = result.structuralTriangles.size();
    for (const PlannedFace& face : plan.virtualFaces) {
        publishFace(face, false);
    }
    for (CanopyApertureRecord& aperture : result.apertures) {
        for (std::size_t& triangle : aperture.closureTriangles) {
            triangle += structuralCount;
        }
    }

    try {
        static_cast<void>(result.body.addMembraneElements(membraneDefinitions));
        const SurfaceGroup contactSurface =
            result.body.surfaceGroup(0, structuralCount);
        static_cast<void>(result.body.addContactSurface(contactSurface, 5.0e-4));
    } catch (const std::exception& error) {
        fail(CanopyPhase::Material, "soft-body-registration", error.what());
    }

    result.audit = {};
    using Edge = std::pair<std::size_t, std::size_t>;
    auto canonicalEdge = [](std::size_t first, std::size_t second) -> Edge {
        return {std::min(first, second), std::max(first, second)};
    };

    std::set<std::size_t> structuralInventory;
    std::vector<std::size_t> structuralClassifications(
        result.body.triangles().size(), 0);
    for (const std::size_t triangle : result.structuralTriangles) {
        if (triangle >= result.body.triangles().size()) {
            ++result.audit.unclassifiedStructuralFaces;
        } else if (!structuralInventory.insert(triangle).second) {
            ++result.audit.multiplyClassifiedStructuralFaces;
        }
    }

    std::map<Edge, std::vector<const CanopyFaceRecord*>> structuralEdges;
    std::set<std::array<std::size_t, 3>> structuralFaceKeys;
    for (const CanopyFaceRecord& face : result.faces) {
        if (face.triangle >= result.body.triangles().size()) {
            ++result.audit.unclassifiedStructuralFaces;
            continue;
        }
        const Triangle& triangle = result.body.triangles()[face.triangle];
        const Vec3& a = result.body.nodes()[triangle.a].position;
        const Vec3& b = result.body.nodes()[triangle.b].position;
        const Vec3& c = result.body.nodes()[triangle.c].position;
        if (!(lengthSquared(cross(b - a, c - a)) > 0.0)) {
            ++result.audit.degenerateFaces;
        }
        if (face.virtualClosure) {
            if (structuralInventory.contains(face.triangle)) {
                ++result.audit.multiplyClassifiedStructuralFaces;
            }
            continue;
        }
        ++structuralClassifications[face.triangle];
        if (face.panelId.empty() ||
            !structuralInventory.contains(face.triangle)) {
            ++result.audit.unclassifiedStructuralFaces;
        }
        std::array<std::size_t, 3> faceKey{triangle.a, triangle.b, triangle.c};
        std::ranges::sort(faceKey);
        if (!structuralFaceKeys.insert(faceKey).second) {
            ++result.audit.multiplyClassifiedStructuralFaces;
        }
        structuralEdges[canonicalEdge(triangle.a, triangle.b)].push_back(&face);
        structuralEdges[canonicalEdge(triangle.b, triangle.c)].push_back(&face);
        structuralEdges[canonicalEdge(triangle.c, triangle.a)].push_back(&face);
    }
    for (const std::size_t triangle : structuralInventory) {
        if (structuralClassifications[triangle] == 0) {
            ++result.audit.unclassifiedStructuralFaces;
        } else if (structuralClassifications[triangle] > 1) {
            ++result.audit.multiplyClassifiedStructuralFaces;
        }
    }
    if (result.faces.size() != result.structuralTriangles.size() +
                                   result.virtualTriangles.size()) {
        ++result.audit.unclassifiedStructuralFaces;
    }

    std::set<Edge> declaredApertureEdges;
    for (const CanopyApertureRecord& aperture : result.apertures) {
        if (aperture.orderedBoundaryNodes.size() < 4) {
            ++result.audit.undeclaredBoundaryEdges;
            continue;
        }
        for (std::size_t i = 0; i < aperture.orderedBoundaryNodes.size(); ++i) {
            declaredApertureEdges.insert(canonicalEdge(
                aperture.orderedBoundaryNodes[i],
                aperture.orderedBoundaryNodes[
                    (i + 1) % aperture.orderedBoundaryNodes.size()]));
        }
    }
    std::set<Edge> declaredMaterialBoundaryEdges;
    for (const CanopyMaterialBoundaryRecord& boundary :
         result.materialBoundaries) {
        if (boundary.orderedNodes.size() < 2) {
            ++result.audit.undeclaredBoundaryEdges;
            continue;
        }
        for (std::size_t i = 1; i < boundary.orderedNodes.size(); ++i) {
            declaredMaterialBoundaryEdges.insert(canonicalEdge(
                boundary.orderedNodes[i - 1], boundary.orderedNodes[i]));
        }
    }
    std::set<Edge> declaredSeamEdges;
    std::set<std::size_t> seamNodes;
    for (const CanopySeamRecord& seam : result.seams) {
        for (std::size_t i = 1; i < seam.orderedNodes.size(); ++i) {
            declaredSeamEdges.insert(
                canonicalEdge(seam.orderedNodes[i - 1], seam.orderedNodes[i]));
        }
        seamNodes.insert(seam.orderedNodes.begin(), seam.orderedNodes.end());
    }

    for (const auto& [candidate, incidences] : structuralEdges) {
        if (incidences.size() == 1) {
            if (!declaredApertureEdges.contains(candidate) &&
                !declaredMaterialBoundaryEdges.contains(candidate)) {
                ++result.audit.undeclaredBoundaryEdges;
            }
            continue;
        }
        std::set<std::string> incidentPanels;
        for (const CanopyFaceRecord* face : incidences) {
            incidentPanels.insert(face->panelId);
        }
        const bool needsDeclaredSeam =
            incidences.size() > 2 || incidentPanels.size() > 1;
        if (needsDeclaredSeam && !declaredSeamEdges.contains(candidate)) {
            ++result.audit.undeclaredNonManifoldEdges;
        }
    }
    for (const Edge& declared : declaredApertureEdges) {
        const auto found = structuralEdges.find(declared);
        if (found == structuralEdges.end() || found->second.size() != 1) {
            ++result.audit.undeclaredBoundaryEdges;
        }
    }
    for (const Edge& declared : declaredMaterialBoundaryEdges) {
        const auto found = structuralEdges.find(declared);
        if (found == structuralEdges.end() || found->second.size() != 1) {
            ++result.audit.undeclaredBoundaryEdges;
        }
    }
    for (const Edge& declared : declaredSeamEdges) {
        const auto found = structuralEdges.find(declared);
        if (found == structuralEdges.end() || found->second.size() < 2) {
            ++result.audit.undeclaredNonManifoldEdges;
        }
    }
    for (const CanopySeamRecord& seam : result.seams) {
        if (seam.firstPanelId == seam.secondPanelId) {
            ++result.audit.undeclaredNonManifoldEdges;
            continue;
        }
        for (std::size_t i = 1; i < seam.orderedNodes.size(); ++i) {
            const Edge candidate = canonicalEdge(seam.orderedNodes[i - 1],
                                                 seam.orderedNodes[i]);
            const auto found = structuralEdges.find(candidate);
            if (found == structuralEdges.end()) {
                ++result.audit.undeclaredNonManifoldEdges;
                continue;
            }
            bool firstOwned = false;
            bool secondOwned = false;
            for (const CanopyFaceRecord* face : found->second) {
                firstOwned = firstOwned || face->panelId == seam.firstPanelId;
                secondOwned = secondOwned || face->panelId == seam.secondPanelId;
            }
            if (!firstOwned || !secondOwned) {
                ++result.audit.undeclaredNonManifoldEdges;
            }
        }
    }

    using GridCell = std::tuple<long long, long long, long long>;
    std::map<GridCell, std::vector<std::size_t>> seamNodesByGridCell;
    const double seamTolerance =
        std::min(1.0e-14, definition.mesh.coordinateTolerance);
    auto gridCoordinate = [&](double value) {
        const long double scaled =
            std::floor(static_cast<long double>(value) / seamTolerance);
        if (scaled < static_cast<long double>(
                         std::numeric_limits<long long>::lowest() + 1) ||
            scaled > static_cast<long double>(
                         std::numeric_limits<long long>::max() - 1)) {
            fail(CanopyPhase::Topology,
                 "structural-audit",
                 "coordinate scale exceeds seam-proximity certification range");
        }
        return static_cast<long long>(scaled);
    };
    for (const std::size_t node : seamNodes) {
        if (node >= result.body.nodes().size()) {
            ++result.audit.coincidentDuplicateSeamNodes;
            continue;
        }
        const Vec3& position = result.body.nodes()[node].position;
        const GridCell key{gridCoordinate(position.x),
                           gridCoordinate(position.y),
                           gridCoordinate(position.z)};
        const auto [gridX, gridY, gridZ] = key;
        bool duplicate = false;
        for (long long dx = -1; dx <= 1 && !duplicate; ++dx) {
            for (long long dy = -1; dy <= 1 && !duplicate; ++dy) {
                for (long long dz = -1; dz <= 1 && !duplicate; ++dz) {
                    const auto found = seamNodesByGridCell.find(
                        GridCell{gridX + dx, gridY + dy, gridZ + dz});
                    if (found == seamNodesByGridCell.end()) {
                        continue;
                    }
                    duplicate = std::ranges::any_of(
                        found->second,
                        [&](std::size_t other) {
                            return other != node &&
                                   length(result.body.nodes()[other].position -
                                          position) <= seamTolerance;
                        });
                }
            }
        }
        if (duplicate) {
            ++result.audit.coincidentDuplicateSeamNodes;
        }
        seamNodesByGridCell[key].push_back(node);
    }
    if (!result.audit.valid()) {
        std::ostringstream details;
        details << "semantic audit failed: unclassified="
                << result.audit.unclassifiedStructuralFaces
                << " multiply-classified="
                << result.audit.multiplyClassifiedStructuralFaces
                << " undeclared-boundary="
                << result.audit.undeclaredBoundaryEdges
                << " undeclared-nonmanifold="
                << result.audit.undeclaredNonManifoldEdges
                << " degenerate=" << result.audit.degenerateFaces
                << " coincident-seam-nodes="
                << result.audit.coincidentDuplicateSeamNodes;
        std::size_t shown = 0;
        for (const auto& [candidate, incidences] : structuralEdges) {
            const bool undeclaredBoundary =
                incidences.size() == 1 &&
                !declaredApertureEdges.contains(candidate) &&
                !declaredMaterialBoundaryEdges.contains(candidate);
            std::set<std::string> incidentPanels;
            for (const CanopyFaceRecord* face : incidences) {
                incidentPanels.insert(face->panelId);
            }
            const bool undeclaredNonManifold =
                (incidences.size() > 2 || incidentPanels.size() > 1) &&
                !declaredSeamEdges.contains(candidate);
            if ((!undeclaredBoundary && !undeclaredNonManifold) || shown++ >= 8) {
                continue;
            }
            details << " edge(" << candidate.first << ',' << candidate.second
                    << ") incidence=" << incidences.size() << " panels=";
            for (const std::string& panel : incidentPanels) {
                details << panel << ',';
            }
        }
        fail(CanopyPhase::Topology, "structural-audit", details.str());
    }

    result.cells.reserve(definition.cellCount);
    for (std::size_t cellIndex = 0; cellIndex < definition.cellCount;
         ++cellIndex) {
        CanopyCellRecord cell;
        cell.id = "cell-" + std::to_string(cellIndex);
        cell.cellIndex = cellIndex;
        cell.provenanceId = definition.provenance.front().id;
        std::vector<Triangle> oriented;
        for (const CanopyFaceRecord& face : result.faces) {
            if (face.panelRole == CanopyPanelRole::Diagonal) {
                continue;
            }
            int orientation = 0;
            if (face.adjacentCells[0] == static_cast<int>(cellIndex)) {
                orientation = 1;
            } else if (face.adjacentCells[1] == static_cast<int>(cellIndex)) {
                orientation = -1;
            }
            if (orientation == 0) {
                continue;
            }
            cell.boundary.push_back({face.triangle,
                                     orientation,
                                     face.semanticId,
                                     face.virtualClosure});
            Triangle triangle = result.body.triangles()[face.triangle];
            if (orientation < 0) {
                std::swap(triangle.b, triangle.c);
            }
            oriented.push_back(triangle);
        }
        cell.diagnostics.topology = validateSurfaceTopology(
            std::span<const Node>{result.body.nodes()},
            std::span<const Triangle>{oriented});
        cell.diagnostics.signedVolume = signedVolume(
            std::span<const Node>{result.body.nodes()},
            std::span<const Triangle>{oriented});
        if (!cell.diagnostics.topology.valid() ||
            !(cell.diagnostics.signedVolume > definition.mesh.volumeFloor) ||
            !finite(cell.diagnostics.signedVolume)) {
            std::ostringstream details;
            details << "cell is not a closed positive directed two-manifold"
                    << " volume=" << cell.diagnostics.signedVolume
                    << " boundary=" << cell.diagnostics.topology.boundaryEdges
                    << " nonmanifold="
                    << cell.diagnostics.topology.nonManifoldEdges
                    << " inconsistent="
                    << cell.diagnostics.topology.inconsistentDirectedEdges
                    << " degenerate="
                    << cell.diagnostics.topology.degenerateFaces;
            std::map<std::pair<std::size_t, std::size_t>, int> edgeCounts;
            for (const Triangle& triangle : oriented) {
                for (const auto edge : {std::pair{triangle.a, triangle.b},
                                        std::pair{triangle.b, triangle.c},
                                        std::pair{triangle.c, triangle.a}}) {
                    edgeCounts[std::minmax(edge.first, edge.second)] += 1;
                }
            }
            int shown = 0;
            for (const auto& [edge, count] : edgeCounts) {
                if (count == 1 && shown++ < 8) {
                    const Vec3& p = result.body.nodes()[edge.first].position;
                    const Vec3& q = result.body.nodes()[edge.second].position;
                    details << " edge(" << p.x << ',' << p.y << ',' << p.z
                            << "->" << q.x << ',' << q.y << ',' << q.z << ')';
                }
            }
            fail(CanopyPhase::Envelope,
                 cell.id,
                 details.str());
        }
        cell.diagnostics.centroid = volumeCentroid(
            std::span<const Node>{result.body.nodes()},
            std::span<const Triangle>{oriented},
            cell.diagnostics.signedVolume);
        const auto [force, moment] = closureResiduals(
            std::span<const Node>{result.body.nodes()},
            std::span<const Triangle>{oriented},
            Vec3{0.5, 0.0, 0.0});
        cell.diagnostics.unitPressureForce = force;
        cell.diagnostics.unitPressureMoment = moment;
        if (length(force) > 1.0e-10 || length(moment) > 1.0e-10) {
            fail(CanopyPhase::Envelope,
                 cell.id,
                 "unit-pressure force or moment does not close");
        }
        double cellMass = 0.0;
        for (std::size_t faceIndex = 0; faceIndex < plan.structural.size();
             ++faceIndex) {
            const PlannedFace& face = plan.structural[faceIndex];
            if (face.materialRole == MaterialRole::Reinforcement) {
                if (plan.panels[face.panel].cellIndex == cellIndex) {
                    cellMass += faceMass[faceIndex];
                }
                continue;
            }
            int ownershipCount = 0;
            bool owns = false;
            for (const int adjacent : face.adjacentCells) {
                ownershipCount += adjacent >= 0 ? 1 : 0;
                owns = owns || adjacent == static_cast<int>(cellIndex);
            }
            if (owns) {
                cellMass += faceMass[faceIndex] /
                            static_cast<double>(ownershipCount);
            }
        }
        cell.diagnostics.materialMass = cellMass;
        result.cells.push_back(std::move(cell));
    }

    // Every aperture loop is exact and every virtual face remains outside fabric ledgers.
    if (result.apertures.size() != definition.apertures.size() ||
        result.virtualTriangles.size() != 2 * result.apertures.size() ||
        result.body.membraneElements().size() != result.structuralTriangles.size() ||
        result.body.contactSurfaces().size() != 1 ||
        result.body.contactSurfaces().front().triangleCount != structuralCount) {
        fail(CanopyPhase::Topology,
             "inventories",
             "material/contact/virtual inventory mismatch");
    }

    // A face owned on both sides is a rib or cross-port closure between two
    // pressurised cells: it must see the difference across it, never a
    // one-sided stamp. Declaring that on the body turns the rule from a
    // comment every caller may ignore into a refusal at the call site.
    for (const CanopyFaceRecord& face : result.faces) {
        if (face.adjacentCells[0] >= 0 && face.adjacentCells[1] >= 0) {
            result.body.declareInteriorPressurePartitions();
            break;
        }
    }
    return result;
}

double canopyFaceGaugeSign(const CanopyFaceRecord& face) {
    // The same signed-zone convention the pneumatic network uses when it
    // registers interfaces; see the header for why the zero cases matter.
    return (face.adjacentCells[0] >= 0 ? 1.0 : 0.0) -
           (face.adjacentCells[1] >= 0 ? 1.0 : 0.0);
}

void setUniformCellPressure(CanopyMesh& mesh, double pressure) {
    mesh.body.setUniformPressureDifference(0.0);
    for (const CanopyFaceRecord& face : mesh.faces) {
        mesh.body.setFacePressureDifference(face.triangle,
                                            canopyFaceGaugeSign(face) *
                                                pressure);
    }
}

CanopyPneumaticLayout buildCanopyPneumaticLayout(
    const CanopyMesh& mesh,
    const CanopyPneumaticInputs& inputs) {
    auto finitePositive = [](double value) {
        return std::isfinite(value) && value > 0.0;
    };
    if (!finitePositive(inputs.ambientPressure) ||
        !finitePositive(inputs.supplyPressure) ||
        !finitePositive(inputs.temperature) ||
        !finitePositive(inputs.inletDischargeCoefficient) ||
        !finitePositive(inputs.crossPortMassConductance) ||
        !std::isfinite(inputs.openingFraction) || inputs.openingFraction < 0.0 ||
        inputs.openingFraction > 1.0) {
        fail(CanopyPhase::Pneumatic,
             "inputs",
             "prescribed pneumatic inputs are invalid");
    }
    const std::size_t cellCount = mesh.definition.cellCount;
    const std::size_t triangleCount = mesh.body.triangles().size();
    if (mesh.cells.size() != cellCount ||
        mesh.apertures.size() != mesh.definition.apertures.size() ||
        !mesh.audit.valid() || cellCount == 0) {
        fail(CanopyPhase::Pneumatic,
             "mesh",
             "semantic canopy records are incomplete or foreign");
    }

    auto checkedInventory = [&](const std::vector<std::size_t>& inventory,
                                const std::string& entity) {
        std::set<std::size_t> result;
        for (const std::size_t triangle : inventory) {
            if (triangle >= triangleCount || !result.insert(triangle).second) {
                fail(CanopyPhase::Pneumatic,
                     entity,
                     "triangle inventory is foreign or duplicated");
            }
        }
        return result;
    };
    const std::set<std::size_t> structural =
        checkedInventory(mesh.structuralTriangles, "structural-inventory");
    const std::set<std::size_t> virtualFaces =
        checkedInventory(mesh.virtualTriangles, "virtual-inventory");
    const std::set<std::size_t> contact =
        checkedInventory(mesh.fabricContactTriangles, "contact-inventory");
    if (structural != contact ||
        std::ranges::any_of(structural, [&](std::size_t triangle) {
            return virtualFaces.contains(triangle);
        }) ||
        mesh.body.membraneElements().size() != structural.size()) {
        fail(CanopyPhase::Pneumatic,
             "inventories",
             "material, contact, membrane, and virtual inventories disagree");
    }

    auto validAdjacentCell = [&](int cell) {
        return cell >= -1 &&
               (cell < 0 || static_cast<std::size_t>(cell) < cellCount);
    };
    std::map<std::size_t, const CanopyFaceRecord*> faceByTriangle;
    for (const CanopyFaceRecord& face : mesh.faces) {
        const std::size_t role = static_cast<std::size_t>(face.panelRole);
        if (face.triangle >= triangleCount || role >= 6 ||
            !faceByTriangle.emplace(face.triangle, &face).second) {
            fail(CanopyPhase::Pneumatic,
                  face.semanticId,
                  "face is foreign, duplicated, or has an invalid role");
        }
        if (!validAdjacentCell(face.adjacentCells[0]) ||
            !validAdjacentCell(face.adjacentCells[1]) ||
            (face.adjacentCells[0] >= 0 &&
             face.adjacentCells[0] == face.adjacentCells[1])) {
            fail(CanopyPhase::Pneumatic,
                 face.semanticId,
                 "face has a foreign or duplicate adjacent cell");
        }
        if (face.panelRole == CanopyPanelRole::Diagonal) {
            if (face.virtualClosure || face.adjacentCells !=
                                           std::array<int, 2>{-1, -1} ||
                !structural.contains(face.triangle)) {
                fail(CanopyPhase::Pneumatic,
                     face.semanticId,
                     "diagonal face has invalid pressure ownership");
            }
            continue;
        }
        if (face.adjacentCells == std::array<int, 2>{-1, -1}) {
            fail(CanopyPhase::Pneumatic,
                 face.semanticId,
                 "pressure face has no finite adjacent cell");
        }
        if (face.virtualClosure) {
            if (!virtualFaces.contains(face.triangle) ||
                structural.contains(face.triangle) ||
                face.apertureId.empty() || !face.materialId.empty() ||
                !face.panelId.empty()) {
                fail(CanopyPhase::Pneumatic,
                     face.semanticId,
                     "virtual closure classification is inconsistent");
            }
        } else if (!structural.contains(face.triangle) ||
                   virtualFaces.contains(face.triangle) ||
                   face.panelId.empty() || face.materialId.empty() ||
                   !face.apertureId.empty()) {
            fail(CanopyPhase::Pneumatic,
                 face.semanticId,
                 "structural pressure-face classification is inconsistent");
        }
    }
    if (faceByTriangle.size() != structural.size() + virtualFaces.size()) {
        fail(CanopyPhase::Pneumatic,
             "faces",
             "semantic faces do not cover the exact triangle inventories");
    }

    std::set<std::string> apertureIds;
    std::set<std::size_t> declaredClosures;
    for (const CanopyApertureRecord& aperture : mesh.apertures) {
        if (aperture.id.empty() || !apertureIds.insert(aperture.id).second ||
            !(aperture.area > 0.0) || !std::isfinite(aperture.area) ||
            !finite(aperture.centroid.x) || !finite(aperture.centroid.y) ||
            !finite(aperture.centroid.z) || !finite(aperture.normal.x) ||
            !finite(aperture.normal.y) || !finite(aperture.normal.z) ||
            std::abs(length(aperture.normal) - 1.0) > 1.0e-10 ||
            aperture.closureTriangles[0] == aperture.closureTriangles[1]) {
            fail(CanopyPhase::Pneumatic,
                 aperture.id,
                 "aperture geometry or identity is invalid");
        }
        if (aperture.kind == CanopyApertureKind::Inlet) {
            if (!validAdjacentCell(aperture.adjacentZones[0]) ||
                aperture.adjacentZones[0] < 0 ||
                aperture.adjacentZones[1] != -1) {
                fail(CanopyPhase::Pneumatic,
                     aperture.id,
                     "inlet has invalid zone ownership");
            }
        } else if (aperture.kind == CanopyApertureKind::CrossPort) {
            if (!validAdjacentCell(aperture.adjacentZones[0]) ||
                !validAdjacentCell(aperture.adjacentZones[1]) ||
                aperture.adjacentZones[0] < 0 ||
                aperture.adjacentZones[1] < 0 ||
                aperture.adjacentZones[0] == aperture.adjacentZones[1]) {
                fail(CanopyPhase::Pneumatic,
                     aperture.id,
                     "cross-port has invalid zone ownership");
            }
        } else {
            fail(CanopyPhase::Pneumatic,
                 aperture.id,
                 "aperture kind is unsupported");
        }

        std::set<std::size_t> lipNodes(aperture.orderedLipNodes.begin(),
                                       aperture.orderedLipNodes.end());
        std::set<std::size_t> boundaryNodes(
            aperture.orderedBoundaryNodes.begin(),
            aperture.orderedBoundaryNodes.end());
        if (lipNodes.size() != 4 ||
            aperture.orderedBoundaryNodes.size() < 4 ||
            boundaryNodes.size() != aperture.orderedBoundaryNodes.size() ||
            *lipNodes.rbegin() >= mesh.body.nodes().size() ||
            *boundaryNodes.rbegin() >= mesh.body.nodes().size()) {
            fail(CanopyPhase::Pneumatic,
                 aperture.id,
                 "aperture lip is incomplete or foreign");
        }
        for (const std::size_t corner : lipNodes) {
            if (!boundaryNodes.contains(corner)) {
                fail(CanopyPhase::Pneumatic,
                     aperture.id,
                     "aperture boundary loop omits a closure corner");
            }
        }
        const Vec3& p0 = mesh.body.nodes()[aperture.orderedLipNodes[0]].position;
        const Vec3& p1 = mesh.body.nodes()[aperture.orderedLipNodes[1]].position;
        const Vec3& p2 = mesh.body.nodes()[aperture.orderedLipNodes[2]].position;
        const Vec3& p3 = mesh.body.nodes()[aperture.orderedLipNodes[3]].position;
        const Vec3 firstAreaVector = 0.5 * cross(p1 - p0, p2 - p0);
        const Vec3 secondAreaVector = 0.5 * cross(p2 - p0, p3 - p0);
        const double certifiedArea =
            length(firstAreaVector) + length(secondAreaVector);
        const Vec3 certifiedNormal =
            normalized(firstAreaVector + secondAreaVector);
        const Vec3 certifiedCentroid =
            (length(firstAreaVector) * ((p0 + p1 + p2) / 3.0) +
             length(secondAreaVector) * ((p0 + p2 + p3) / 3.0)) /
            certifiedArea;
        if (!(dot(certifiedNormal, aperture.normal) > 1.0 - 1.0e-10) ||
            std::abs(certifiedArea - aperture.area) >
                1.0e-10 * std::max(1.0, aperture.area) ||
            length(certifiedCentroid - aperture.centroid) > 1.0e-10) {
            fail(CanopyPhase::Pneumatic,
                 aperture.id,
                 "aperture orientation, area, or centroid is inconsistent");
        }

        std::set<std::size_t> closureNodes;
        for (const std::size_t closure : aperture.closureTriangles) {
            const auto found = faceByTriangle.find(closure);
            if (closure >= triangleCount || !virtualFaces.contains(closure) ||
                !declaredClosures.insert(closure).second ||
                found == faceByTriangle.end() || !found->second->virtualClosure ||
                found->second->apertureId != aperture.id ||
                found->second->adjacentCells != aperture.adjacentZones) {
                fail(CanopyPhase::Pneumatic,
                     aperture.id,
                     "closure face association is invalid");
            }
            const Triangle& triangle = mesh.body.triangles()[closure];
            for (const std::size_t node :
                 {triangle.a, triangle.b, triangle.c}) {
                if (!lipNodes.contains(node)) {
                    fail(CanopyPhase::Pneumatic,
                         aperture.id,
                         "closure face uses a non-lip node");
                }
                closureNodes.insert(node);
            }
        }
        if (closureNodes != lipNodes) {
            fail(CanopyPhase::Pneumatic,
                 aperture.id,
                 "closure faces do not span the exact aperture lip");
        }
    }
    if (declaredClosures != virtualFaces) {
        fail(CanopyPhase::Pneumatic,
             "virtual-inventory",
             "virtual closures are not in one-to-one aperture association");
    }

    std::vector<double> certifiedCellVolumes(cellCount, 0.0);
    for (std::size_t cellIndex = 0; cellIndex < cellCount; ++cellIndex) {
        const CanopyCellRecord& cell = mesh.cells[cellIndex];
        if (cell.cellIndex != cellIndex || cell.id.empty()) {
            fail(CanopyPhase::Pneumatic,
                 cell.id,
                 "cell identity or stable ordering is invalid");
        }
        std::map<std::size_t, int> expectedBoundary;
        for (const CanopyFaceRecord& face : mesh.faces) {
            if (face.panelRole == CanopyPanelRole::Diagonal) {
                continue;
            }
            if (face.adjacentCells[0] == static_cast<int>(cellIndex)) {
                expectedBoundary.emplace(face.triangle, 1);
            } else if (face.adjacentCells[1] == static_cast<int>(cellIndex)) {
                expectedBoundary.emplace(face.triangle, -1);
            }
        }
        std::map<std::size_t, int> actualBoundary;
        std::vector<Triangle> oriented;
        oriented.reserve(cell.boundary.size());
        for (const CanopyBoundaryFace& boundary : cell.boundary) {
            const auto face = faceByTriangle.find(boundary.triangle);
            const auto expected = expectedBoundary.find(boundary.triangle);
            if (face == faceByTriangle.end() ||
                expected == expectedBoundary.end() ||
                boundary.orientation != expected->second ||
                boundary.semanticId != face->second->semanticId ||
                boundary.virtualClosure != face->second->virtualClosure ||
                !actualBoundary.emplace(boundary.triangle,
                                        boundary.orientation).second) {
                fail(CanopyPhase::Pneumatic,
                     cell.id,
                     "cell boundary record is foreign or inconsistent");
            }
            Triangle triangle = mesh.body.triangles()[boundary.triangle];
            if (boundary.orientation < 0) {
                std::swap(triangle.b, triangle.c);
            }
            oriented.push_back(triangle);
        }
        if (actualBoundary != expectedBoundary ||
            !validateSurfaceTopology(
                 std::span<const Node>{mesh.body.nodes()},
                 std::span<const Triangle>{oriented}).valid()) {
            fail(CanopyPhase::Pneumatic,
                 cell.id,
                 "cell boundary is not the exact closed directed envelope");
        }
        certifiedCellVolumes[cellIndex] = signedVolume(
            std::span<const Node>{mesh.body.nodes()},
            std::span<const Triangle>{oriented});
        if (!(certifiedCellVolumes[cellIndex] >
              mesh.definition.mesh.volumeFloor) ||
            !finite(certifiedCellVolumes[cellIndex])) {
            fail(CanopyPhase::Pneumatic,
                 cell.id,
                 "cell volume is not positively recertified");
        }
    }

    CanopyPneumaticLayout result;
    result.inputs = inputs;
    result.network = std::make_unique<PneumaticNetwork>();
    result.limitation =
        "Static Stage 3 compatibility only: virtual control-volume closures are not fabric; "
        "no resolved inlet momentum or external airflow is present.";
    try {
        std::vector<PneumaticZoneHandle> cells;
        cells.reserve(mesh.cells.size());
        for (std::size_t cellIndex = 0; cellIndex < mesh.cells.size();
             ++cellIndex) {
            const CanopyCellRecord& cell = mesh.cells[cellIndex];
            cells.push_back(result.network->addGasCell(
                {1.0, inputs.temperature, 1.0e-12,
                  mesh.definition.mesh.volumeFloor}));
            result.cellZoneIndices.push_back(cells.back().index());
            if (!(certifiedCellVolumes[cellIndex] >
                  mesh.definition.mesh.volumeFloor)) {
                fail(CanopyPhase::Pneumatic,
                     cell.id,
                     "cell volume is not certified");
            }
        }
        const PneumaticZoneHandle ambient = result.network->addReservoir(
            {inputs.ambientPressure,
             inputs.temperature,
             QuantityProvenance::Prescribed,
             QuantityProvenance::Prescribed});
        const PneumaticZoneHandle supply = result.network->addReservoir(
            {inputs.supplyPressure,
             inputs.temperature,
             QuantityProvenance::Prescribed,
             QuantityProvenance::Prescribed});
        result.ambientReservoirIndex = ambient.index();
        result.supplyReservoirIndex = supply.index();

        std::set<std::size_t> registeredTriangles;
        for (const CanopyFaceRecord& face : mesh.faces) {
            if (face.panelRole == CanopyPanelRole::Diagonal) {
                continue;
            }
            if (!registeredTriangles.insert(face.triangle).second) {
                fail(CanopyPhase::Pneumatic,
                     face.semanticId,
                     "duplicate pressure interface registration");
            }
            const bool inletClosure = face.virtualClosure &&
                std::ranges::any_of(mesh.apertures,
                                    [&](const CanopyApertureRecord& aperture) {
                                        return aperture.id == face.apertureId &&
                                               aperture.kind ==
                                                   CanopyApertureKind::Inlet;
                                    });
            const PneumaticZoneHandle& exterior = inletClosure ? supply : ambient;
            const PneumaticZoneHandle& negative =
                face.adjacentCells[0] >= 0
                    ? cells[static_cast<std::size_t>(face.adjacentCells[0])]
                    : exterior;
            const PneumaticZoneHandle& positive =
                face.adjacentCells[1] >= 0
                    ? cells[static_cast<std::size_t>(face.adjacentCells[1])]
                    : exterior;
            const PressureInterfaceHandle interfaceHandle =
                result.network->addPressureInterface(
                    mesh.body, face.triangle, negative, positive);
            result.pressureInterfaceIndices.push_back(interfaceHandle.index());
        }
        for (const CanopyApertureRecord& aperture : mesh.apertures) {
            if (!(aperture.area > 0.0) ||
                aperture.closureTriangles[0] >= mesh.body.triangles().size() ||
                aperture.closureTriangles[1] >= mesh.body.triangles().size()) {
                fail(CanopyPhase::Pneumatic,
                     aperture.id,
                     "missing closure or invalid aperture geometry");
            }
            CanopyPneumaticPortRecord port;
            port.id = aperture.id;
            port.kind = aperture.kind;
            port.adjacentZones = aperture.adjacentZones;
            port.area = aperture.area;
            port.centroid = aperture.centroid;
            port.normal = aperture.normal;
            port.closureTriangles = aperture.closureTriangles;
            port.prescribedOpening = inputs.openingFraction;
            port.provenanceId = aperture.provenanceId;
            port.limitation =
                "virtual control-volume closure; excluded from fabric/material/contact; "
                "no resolved inlet momentum or external flow";
            if (aperture.kind == CanopyApertureKind::Inlet) {
                if (aperture.adjacentZones[0] < 0) {
                    fail(CanopyPhase::Pneumatic,
                         aperture.id,
                         "inlet has no finite adjacent cell");
                }
                const PneumaticPortHandle handle = result.network->addOrificePort(
                    cells[static_cast<std::size_t>(aperture.adjacentZones[0])],
                    supply,
                    {inputs.inletDischargeCoefficient,
                     aperture.area,
                     inputs.openingFraction});
                port.networkPortIndex = handle.index();
                port.prescribedCoefficient = inputs.inletDischargeCoefficient;
            } else {
                if (aperture.adjacentZones[0] < 0 ||
                    aperture.adjacentZones[1] < 0) {
                    fail(CanopyPhase::Pneumatic,
                         aperture.id,
                         "cross-port does not have two adjacent cells");
                }
                const PneumaticPortHandle handle =
                    result.network->addConductancePort(
                        cells[static_cast<std::size_t>(aperture.adjacentZones[0])],
                        cells[static_cast<std::size_t>(aperture.adjacentZones[1])],
                        {inputs.crossPortMassConductance,
                         inputs.openingFraction});
                port.networkPortIndex = handle.index();
                port.prescribedCoefficient = inputs.crossPortMassConductance;
            }
            result.ports.push_back(std::move(port));
        }
        for (const PneumaticZoneHandle& cell : cells) {
            result.network->initializeCellMassFromPressure(
                mesh.body, cell, inputs.ambientPressure);
        }
        result.network->validate(mesh.body);
        result.massResidual = result.network->diagnostics().massResidual;
        if (!result.network->diagnostics().topologyValid ||
            std::abs(result.massResidual) > 1.0e-12) {
            fail(CanopyPhase::Pneumatic,
                 "network",
                 "topology or mass-ledger closure failed");
        }
    } catch (const CanopyError&) {
        throw;
    } catch (const std::exception& error) {
        fail(CanopyPhase::Pneumatic, "network-registration", error.what());
    }
    return result;
}

} // namespace softwing
