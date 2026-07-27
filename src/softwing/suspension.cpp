#include "softwing/suspension.h"

#ifdef SOFTWING_AERODYNAMIC_TEST_ACCESS
#include "aerodynamic_test_access.h"
#endif

#include <algorithm>
#include <bit>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <numbers>
#include <ranges>
#include <set>
#include <sstream>
#include <unordered_map>
#include <utility>

namespace softwing {
namespace {

constexpr std::size_t kMaximumRecords = 10000;
constexpr double kFiniteTolerance = 1.0e-14;

[[noreturn]] void fail(SuspensionPhase phase,
                       const std::string& entity,
                       const std::string& message) {
    throw SuspensionError(phase, entity, message);
}

bool finite(double value) { return std::isfinite(value); }
bool finite(const Vec2& value) { return finite(value.x) && finite(value.y); }
bool finite(const Vec3& value) {
    return finite(value.x) && finite(value.y) && finite(value.z);
}
bool finite(const Quaternion& value) {
    return finite(value.w) && finite(value.x) && finite(value.y) &&
           finite(value.z);
}
bool finite(const Mat3& value) {
    return std::ranges::all_of(value.values,
                               [](double entry) { return finite(entry); });
}

double quaternionNormSquared(const Quaternion& value) {
    return value.w * value.w + value.x * value.x + value.y * value.y +
           value.z * value.z;
}

bool validUtf8(std::string_view text) {
    std::size_t i = 0;
    while (i < text.size()) {
        const auto first = static_cast<unsigned char>(text[i]);
        if (first <= 0x7fU) {
            if (first < 0x20U && first != '\t' && first != '\n' &&
                first != '\r') {
                return false;
            }
            ++i;
            continue;
        }
        std::size_t continuation = 0;
        unsigned int code = 0;
        if ((first & 0xe0U) == 0xc0U) {
            continuation = 1;
            code = first & 0x1fU;
            if (code == 0U) return false;
        } else if ((first & 0xf0U) == 0xe0U) {
            continuation = 2;
            code = first & 0x0fU;
        } else if ((first & 0xf8U) == 0xf0U) {
            continuation = 3;
            code = first & 0x07U;
        } else {
            return false;
        }
        if (i + continuation >= text.size()) return false;
        for (std::size_t j = 1; j <= continuation; ++j) {
            const auto next = static_cast<unsigned char>(text[i + j]);
            if ((next & 0xc0U) != 0x80U) return false;
            code = (code << 6U) | (next & 0x3fU);
        }
        if ((continuation == 1 && code < 0x80U) ||
            (continuation == 2 && code < 0x800U) ||
            (continuation == 3 && code < 0x10000U) ||
            code > 0x10ffffU || (code >= 0xd800U && code <= 0xdfffU)) {
            return false;
        }
        i += continuation + 1;
    }
    return true;
}

void requireText(std::string_view value,
                 SuspensionPhase phase,
                 const std::string& entity,
                 bool allowEmpty = false) {
    if ((!allowEmpty && value.empty()) || !validUtf8(value)) {
        fail(phase, entity, "missing or invalid UTF-8 text");
    }
}

template <typename Record>
void requireUniqueIds(const std::vector<Record>& records,
                      const std::string& kind) {
    std::set<std::string> ids;
    for (const Record& record : records) {
        requireText(record.id, SuspensionPhase::Validation, kind);
        if (!ids.insert(record.id).second) {
            fail(SuspensionPhase::Validation, record.id, "duplicate " + kind);
        }
    }
}

const SuspensionProvenance& provenanceById(
    const SuspensionDefinition& definition,
    const std::string& id,
    const std::string& entity) {
    const auto found = std::ranges::find_if(
        definition.provenance,
        [&](const SuspensionProvenance& record) { return record.id == id; });
    if (found == definition.provenance.end()) {
        fail(SuspensionPhase::Validation,
             entity,
             "foreign provenance reference: " + id);
    }
    return *found;
}

template <typename Enum>
int enumValue(Enum value) {
    return static_cast<int>(value);
}

bool validSide(SuspensionSide side) {
    return side == SuspensionSide::Left || side == SuspensionSide::Right ||
           side == SuspensionSide::Centre;
}

bool validControlKind(SuspensionControlKind kind) {
    return kind == SuspensionControlKind::Brake ||
           kind == SuspensionControlKind::Accelerator ||
           kind == SuspensionControlKind::Riser ||
           kind == SuspensionControlKind::BigEar ||
           kind == SuspensionControlKind::WeightShift;
}

bool validControlTargetKind(SuspensionControlTargetKind kind) {
    return kind == SuspensionControlTargetKind::SegmentRestLength ||
           kind == SuspensionControlTargetKind::HangPointTravel;
}

bool validGroundMode(PayloadGroundMode mode) {
    return mode == PayloadGroundMode::Free ||
           mode == PayloadGroundMode::Anchored ||
           mode == PayloadGroundMode::SupportPlane;
}

SuspensionSide endpointSide(const SuspensionDefinition& definition,
                            const SuspensionEndpoint& endpoint) {
    if (endpoint.kind == SuspensionEndpointKind::Attachment) {
        const auto found = std::ranges::find_if(
            definition.attachments,
            [&](const auto& value) { return value.id == endpoint.id; });
        if (found == definition.attachments.end()) {
            fail(SuspensionPhase::Validation, endpoint.id,
                 "foreign attachment endpoint");
        }
        return found->side;
    }
    if (endpoint.kind == SuspensionEndpointKind::Junction) {
        const auto found = std::ranges::find_if(
            definition.junctions,
            [&](const auto& value) { return value.id == endpoint.id; });
        if (found == definition.junctions.end()) {
            fail(SuspensionPhase::Validation, endpoint.id,
                 "foreign junction endpoint");
        }
        return found->side;
    }
    if (endpoint.kind == SuspensionEndpointKind::HangPoint) {
        const auto found = std::ranges::find_if(
            definition.payload.hangPoints,
            [&](const auto& value) { return value.id == endpoint.id; });
        if (found == definition.payload.hangPoints.end()) {
            fail(SuspensionPhase::Validation, endpoint.id,
                 "foreign hang-point endpoint");
        }
        return found->side;
    }
    fail(SuspensionPhase::Validation, endpoint.id,
         "unsupported endpoint kind");
}

std::string endpointKey(const SuspensionEndpoint& endpoint) {
    return std::to_string(enumValue(endpoint.kind)) + ":" + endpoint.id;
}

const char* controlTargetKindName(SuspensionControlTargetKind kind) {
    switch (kind) {
    case SuspensionControlTargetKind::SegmentRestLength: return "segment";
    case SuspensionControlTargetKind::HangPointTravel: return "hang";
    }
    return "invalid";
}

SuspensionControlTargetKind parseControlTargetKind(const std::string& value) {
    if (value == "segment")
        return SuspensionControlTargetKind::SegmentRestLength;
    if (value == "hang")
        return SuspensionControlTargetKind::HangPointTravel;
    fail(SuspensionPhase::Parse, "control-target-kind",
         "unsupported control target kind");
}

SuspensionSide parseSide(const std::string& value) {
    if (value == "left") return SuspensionSide::Left;
    if (value == "right") return SuspensionSide::Right;
    if (value == "centre") return SuspensionSide::Centre;
    fail(SuspensionPhase::Parse, "side", "unsupported side");
}

SuspensionEndpointKind parseEndpointKind(const std::string& value) {
    if (value == "attachment") return SuspensionEndpointKind::Attachment;
    if (value == "junction") return SuspensionEndpointKind::Junction;
    if (value == "hang-point") return SuspensionEndpointKind::HangPoint;
    fail(SuspensionPhase::Parse, "endpoint-kind", "unsupported endpoint kind");
}

SuspensionControlKind parseControlKind(const std::string& value) {
    if (value == "brake") return SuspensionControlKind::Brake;
    if (value == "accelerator") return SuspensionControlKind::Accelerator;
    if (value == "riser") return SuspensionControlKind::Riser;
    if (value == "big-ear") return SuspensionControlKind::BigEar;
    if (value == "weight-shift") return SuspensionControlKind::WeightShift;
    fail(SuspensionPhase::Parse, "control-kind", "unsupported control kind");
}

PayloadGroundMode parseGroundMode(const std::string& value) {
    if (value == "free") return PayloadGroundMode::Free;
    if (value == "anchored") return PayloadGroundMode::Anchored;
    if (value == "support-plane") return PayloadGroundMode::SupportPlane;
    fail(SuspensionPhase::Parse, "ground-mode", "unsupported ground mode");
}

std::size_t checkedCount(std::istream& input, const std::string& entity) {
    std::size_t count = 0;
    if (!(input >> count) || count > kMaximumRecords) {
        fail(SuspensionPhase::Parse, entity,
             "missing or excessive explicit count");
    }
    return count;
}

void expectToken(std::istream& input,
                 const std::string& expected,
                 const std::string& entity) {
    std::string actual;
    if (!(input >> actual) || actual != expected) {
        fail(SuspensionPhase::Parse,
             entity,
             "expected ordered field " + expected);
    }
}

std::string readQuoted(std::istream& input, const std::string& entity) {
    std::string value;
    if (!(input >> std::quoted(value)) || !validUtf8(value)) {
        fail(SuspensionPhase::Parse, entity, "invalid escaped UTF-8 text");
    }
    return value;
}

double readDouble(std::istream& input, const std::string& entity) {
    double value = 0.0;
    if (!(input >> value) || !finite(value)) {
        fail(SuspensionPhase::Parse, entity, "invalid finite number");
    }
    return value;
}

Mat3 diagonalMatrix(double x, double y, double z) {
    return {{x, 0.0, 0.0, 0.0, y, 0.0, 0.0, 0.0, z}};
}

double scaledRelative(double first, double second) {
    return std::abs(first - second) /
           std::max({1.0e-30, std::abs(first), std::abs(second)});
}

} // namespace

SuspensionError::SuspensionError(SuspensionPhase phase,
                                 std::string entity,
                                 const std::string& message)
    : std::runtime_error(std::string(suspensionPhaseName(phase)) + ":" +
                         entity + ": " + message),
      phase_(phase),
      entity_(std::move(entity)) {}

const char* suspensionPhaseName(SuspensionPhase phase) {
    switch (phase) {
    case SuspensionPhase::Parse: return "parse";
    case SuspensionPhase::Validation: return "validation";
    case SuspensionPhase::AttachmentResolution: return "attachment-resolution";
    case SuspensionPhase::GraphConstruction: return "graph-construction";
    case SuspensionPhase::Control: return "control";
    case SuspensionPhase::Prediction: return "prediction";
    case SuspensionPhase::LineSolve: return "line-solve";
    case SuspensionPhase::GroundSolve: return "ground-solve";
    case SuspensionPhase::Certification: return "certification";
    case SuspensionPhase::Diagnostics: return "diagnostics";
    }
    return "invalid";
}

const char* suspensionSideName(SuspensionSide side) {
    switch (side) {
    case SuspensionSide::Left: return "left";
    case SuspensionSide::Right: return "right";
    case SuspensionSide::Centre: return "centre";
    }
    return "invalid";
}

const char* suspensionEndpointKindName(SuspensionEndpointKind kind) {
    switch (kind) {
    case SuspensionEndpointKind::Attachment: return "attachment";
    case SuspensionEndpointKind::Junction: return "junction";
    case SuspensionEndpointKind::HangPoint: return "hang-point";
    }
    return "invalid";
}

const char* suspensionControlKindName(SuspensionControlKind kind) {
    switch (kind) {
    case SuspensionControlKind::Brake: return "brake";
    case SuspensionControlKind::Accelerator: return "accelerator";
    case SuspensionControlKind::Riser: return "riser";
    case SuspensionControlKind::BigEar: return "big-ear";
    case SuspensionControlKind::WeightShift: return "weight-shift";
    }
    return "invalid";
}

const char* payloadGroundModeName(PayloadGroundMode mode) {
    switch (mode) {
    case PayloadGroundMode::Free: return "free";
    case PayloadGroundMode::Anchored: return "anchored";
    case PayloadGroundMode::SupportPlane: return "support-plane";
    }
    return "invalid";
}

Mat3 transpose(const Mat3& matrix) {
    Mat3 result{};
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            result(row, column) = matrix(column, row);
    return result;
}

double determinant(const Mat3& m) {
    return m(0, 0) * (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1)) -
           m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)) +
           m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));
}

Mat3 inverse(const Mat3& m) {
    const double scale = std::abs(*std::ranges::max_element(
        m.values, {}, [](double value) { return std::abs(value); }));
    if (!(scale > 0.0) || !finite(scale)) {
        fail(SuspensionPhase::Validation, "inertia", "singular matrix");
    }
    Mat3 normalized = m;
    for (double& value : normalized.values) value /= scale;
    const double det = determinant(normalized);
    if (!finite(det) || std::abs(det) <= kFiniteTolerance) {
        fail(SuspensionPhase::Validation, "inertia", "singular matrix");
    }
    Mat3 result{{
        normalized(1, 1) * normalized(2, 2) -
            normalized(1, 2) * normalized(2, 1),
        normalized(0, 2) * normalized(2, 1) -
            normalized(0, 1) * normalized(2, 2),
        normalized(0, 1) * normalized(1, 2) -
            normalized(0, 2) * normalized(1, 1),
        normalized(1, 2) * normalized(2, 0) -
            normalized(1, 0) * normalized(2, 2),
        normalized(0, 0) * normalized(2, 2) -
            normalized(0, 2) * normalized(2, 0),
        normalized(0, 2) * normalized(1, 0) -
            normalized(0, 0) * normalized(1, 2),
        normalized(1, 0) * normalized(2, 1) -
            normalized(1, 1) * normalized(2, 0),
        normalized(0, 1) * normalized(2, 0) -
            normalized(0, 0) * normalized(2, 1),
        normalized(0, 0) * normalized(1, 1) -
            normalized(0, 1) * normalized(1, 0)}};
    for (double& value : result.values) value /= det * scale;
    return result;
}

Vec3 operator*(const Mat3& m, const Vec3& v) {
    return {m(0, 0) * v.x + m(0, 1) * v.y + m(0, 2) * v.z,
            m(1, 0) * v.x + m(1, 1) * v.y + m(1, 2) * v.z,
            m(2, 0) * v.x + m(2, 1) * v.y + m(2, 2) * v.z};
}

Mat3 operator*(const Mat3& a, const Mat3& b) {
    Mat3 result{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            for (std::size_t inner = 0; inner < 3; ++inner)
                result(row, column) += a(row, inner) * b(inner, column);
    return result;
}

Quaternion normalizedCanonical(const Quaternion& q) {
    const double norm2 = quaternionNormSquared(q);
    if (!finite(q) || !(norm2 > 1.0e-28)) {
        fail(SuspensionPhase::Validation, "orientation",
             "quaternion is non-finite or near zero");
    }
    const double invNorm = 1.0 / std::sqrt(norm2);
    Quaternion result{q.w * invNorm, q.x * invNorm, q.y * invNorm,
                      q.z * invNorm};
    const bool flip = result.w < 0.0 ||
        (result.w == 0.0 &&
         (result.x < 0.0 ||
          (result.x == 0.0 &&
           (result.y < 0.0 || (result.y == 0.0 && result.z < 0.0)))));
    if (flip) {
        result.w = -result.w;
        result.x = -result.x;
        result.y = -result.y;
        result.z = -result.z;
    }
    return result;
}

Quaternion operator*(const Quaternion& a, const Quaternion& b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

Mat3 rotationMatrix(const Quaternion& orientation) {
    const Quaternion q = normalizedCanonical(orientation);
    const double xx = q.x * q.x;
    const double yy = q.y * q.y;
    const double zz = q.z * q.z;
    const double xy = q.x * q.y;
    const double xz = q.x * q.z;
    const double yz = q.y * q.z;
    const double wx = q.w * q.x;
    const double wy = q.w * q.y;
    const double wz = q.w * q.z;
    return {{1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),
             2.0 * (xz + wy), 2.0 * (xy + wz),
             1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
             2.0 * (xz - wy), 2.0 * (yz + wx),
             1.0 - 2.0 * (xx + yy)}};
}

Vec3 rotate(const Quaternion& orientation, const Vec3& vector) {
    return rotationMatrix(orientation) * vector;
}

Quaternion rotationIncrement(const Vec3& rotationVector) {
    if (!finite(rotationVector)) {
        fail(SuspensionPhase::Prediction, "rotation-increment",
             "non-finite rotation vector");
    }
    const double angle = length(rotationVector);
    if (angle < 1.0e-12) {
        return normalizedCanonical(
            {1.0, 0.5 * rotationVector.x, 0.5 * rotationVector.y,
             0.5 * rotationVector.z});
    }
    const double half = 0.5 * angle;
    const double scale = std::sin(half) / angle;
    return {std::cos(half), rotationVector.x * scale,
            rotationVector.y * scale, rotationVector.z * scale};
}

SuspensionDefinition validateAndNormalizeSuspensionDefinition(
    const SuspensionDefinition& input) {
    SuspensionDefinition definition = input;
    if (definition.schemaMajor != 1) {
        fail(SuspensionPhase::Validation, "schema-major",
             "only SOFTWING_SUSPENSION version 1 is supported");
    }
    requireText(definition.identifier, SuspensionPhase::Validation,
                "identifier");
    requireText(definition.description, SuspensionPhase::Validation,
                "description");
    if (definition.unitsFrameTag != suspensionStage5FrameTag) {
        fail(SuspensionPhase::Validation, "units-frame",
             "unsupported SI/frame tag");
    }
    if (definition.provenance.empty() ||
        definition.provenance.size() > kMaximumRecords) {
        fail(SuspensionPhase::Validation, "provenance",
             "non-empty bounded provenance is required");
    }
    requireUniqueIds(definition.provenance, "provenance");
    for (const auto& record : definition.provenance)
        requireText(record.source, SuspensionPhase::Validation, record.id);

    if (definition.attachments.empty() ||
        definition.attachments.size() > kMaximumRecords ||
        definition.junctions.size() > kMaximumRecords ||
        definition.segments.empty() ||
        definition.segments.size() > kMaximumRecords ||
        definition.controls.size() > kMaximumRecords ||
        definition.payload.hangPoints.size() > kMaximumRecords ||
        definition.payload.supportPoints.size() > kMaximumRecords) {
        fail(SuspensionPhase::Validation, "counts",
             "definition record counts are empty or excessive");
    }
    requireUniqueIds(definition.attachments, "attachment");
    requireUniqueIds(definition.junctions, "junction");
    requireUniqueIds(definition.payload.hangPoints, "hang-point");
    requireUniqueIds(definition.payload.supportPoints, "support-point");
    requireUniqueIds(definition.segments, "segment");
    requireUniqueIds(definition.controls, "control");

    std::set<std::string> endpointIds;
    const auto retainEndpointId = [&](const std::string& id) {
        if (!endpointIds.insert(id).second) {
            fail(SuspensionPhase::Validation, id,
                 "endpoint id is ambiguous across semantic kinds");
        }
    };
    for (const auto& attachment : definition.attachments) {
        retainEndpointId(attachment.id);
        requireText(attachment.panelId, SuspensionPhase::Validation,
                    attachment.id);
        requireText(attachment.seamId, SuspensionPhase::Validation,
                    attachment.id, true);
        if (!finite(attachment.chart) || !validSide(attachment.side)) {
            fail(SuspensionPhase::Validation, attachment.id,
                 "invalid attachment chart or side");
        }
        static_cast<void>(provenanceById(definition, attachment.provenanceId,
                                         attachment.id));
    }
    for (const auto& junction : definition.junctions) {
        retainEndpointId(junction.id);
        if (!finite(junction.initialWorldPosition) ||
            !(junction.mass > 0.0) || !finite(junction.mass) ||
            !validSide(junction.side)) {
            fail(SuspensionPhase::Validation, junction.id,
                 "junction requires finite position and positive mass");
        }
        static_cast<void>(provenanceById(definition, junction.provenanceId,
                                         junction.id));
    }

    RigidPayloadDefinition& payload = definition.payload;
    if (!(payload.mass > 0.0) || !finite(payload.mass) ||
        !finite(payload.centreOfMassLocal) || !finite(payload.inertiaBody) ||
        !finite(payload.initialState.centreOfMassWorld) ||
        !finite(payload.initialState.linearVelocity) ||
        !finite(payload.initialState.angularVelocity) ||
        !finite(payload.initialState.orientation)) {
        fail(SuspensionPhase::Validation, "payload",
             "payload state, mass, and inertia must be finite and positive");
    }
    payload.initialState.orientation =
        normalizedCanonical(payload.initialState.orientation);
    const auto asymmetric = [&](std::size_t row, std::size_t column) {
        const double first = payload.inertiaBody(row, column);
        const double second = payload.inertiaBody(column, row);
        const double pairScale =
            std::max({1.0e-30, std::abs(first), std::abs(second)});
        return std::abs(first - second) > 1.0e-12 * pairScale;
    };
    if (asymmetric(0, 1) || asymmetric(0, 2) || asymmetric(1, 2)) {
        fail(SuspensionPhase::Validation, "payload-inertia",
             "body inertia must be symmetric");
    }
    const double minor1 = payload.inertiaBody(0, 0);
    const double minor2 = payload.inertiaBody(0, 0) * payload.inertiaBody(1, 1) -
                          payload.inertiaBody(0, 1) * payload.inertiaBody(1, 0);
    if (!(minor1 > 0.0) || !(minor2 > 0.0) ||
        !(determinant(payload.inertiaBody) > 0.0)) {
        fail(SuspensionPhase::Validation, "payload-inertia",
             "body inertia must be symmetric positive definite");
    }
    static_cast<void>(inverse(payload.inertiaBody));
    static_cast<void>(provenanceById(definition, payload.provenanceId,
                                     "payload"));
    if (payload.hangPoints.empty()) {
        fail(SuspensionPhase::Validation, "payload-hang-points",
             "at least one hang point is required");
    }
    for (const auto& point : payload.hangPoints) {
        retainEndpointId(point.id);
        if (!finite(point.localPosition) || !validSide(point.side)) {
            fail(SuspensionPhase::Validation, point.id,
                 "invalid hang-point geometry or side");
        }
        static_cast<void>(provenanceById(definition, point.provenanceId,
                                         point.id));
    }
    for (const auto& point : payload.supportPoints) {
        if (!finite(point.localPosition) || !validSide(point.side)) {
            fail(SuspensionPhase::Validation, point.id,
                 "invalid support-point geometry or side");
        }
        static_cast<void>(provenanceById(definition, point.provenanceId,
                                         point.id));
    }

    std::set<std::pair<std::string, std::string>> unorderedPairs;
    std::map<std::string, std::string> parent;
    std::map<std::string, std::size_t> incoming;
    for (const auto& segment : definition.segments) {
        requireText(segment.from.id, SuspensionPhase::Validation, segment.id);
        requireText(segment.to.id, SuspensionPhase::Validation, segment.id);
        static_cast<void>(endpointSide(definition, segment.from));
        static_cast<void>(endpointSide(definition, segment.to));
        if (segment.from.kind == SuspensionEndpointKind::HangPoint ||
            segment.to.kind == SuspensionEndpointKind::Attachment ||
            segment.from.id == segment.to.id) {
            fail(SuspensionPhase::Validation, segment.id,
                 "invalid directed endpoint kinds");
        }
        if (!(segment.restLength > 0.0) || !finite(segment.restLength) ||
            !(segment.axialStiffness > 0.0) ||
            !finite(segment.axialStiffness) ||
            segment.axialDamping < 0.0 || !finite(segment.axialDamping) ||
            !validSide(segment.side)) {
            fail(SuspensionPhase::Validation, segment.id,
                 "line length/stiffness must be positive and damping non-negative");
        }
        const SuspensionSide fromSide = endpointSide(definition, segment.from);
        const SuspensionSide toSide = endpointSide(definition, segment.to);
        if ((fromSide != SuspensionSide::Centre && segment.side != fromSide) ||
            (toSide != SuspensionSide::Centre && segment.side != toSide)) {
            fail(SuspensionPhase::Validation, segment.id,
                 "segment side does not match endpoint side");
        }
        std::set<std::string> groups;
        for (const auto& group : segment.groups) {
            requireText(group, SuspensionPhase::Validation, segment.id);
            if (!groups.insert(group).second)
                fail(SuspensionPhase::Validation, segment.id,
                     "duplicate line group");
        }
        if (segment.groups.empty() ||
            segment.groups.size() > kMaximumRecords) {
            fail(SuspensionPhase::Validation, segment.id,
                 "at least one line group is required");
        }
        static_cast<void>(provenanceById(definition, segment.provenanceId,
                                         segment.id));
        std::array<std::string, 2> pair{endpointKey(segment.from),
                                        endpointKey(segment.to)};
        std::ranges::sort(pair);
        if (!unorderedPairs.insert({pair[0], pair[1]}).second) {
            fail(SuspensionPhase::Validation, segment.id,
                 "duplicate or reverse-duplicate segment");
        }
        const std::string fromKey = endpointKey(segment.from);
        const std::string toKey = endpointKey(segment.to);
        if (!parent.emplace(fromKey, toKey).second) {
            fail(SuspensionPhase::Validation, segment.id,
                 "endpoint has multiple downstream parents");
        }
        ++incoming[toKey];
    }

    std::set<std::string> reachedJunctions;
    for (const auto& attachment : definition.attachments) {
        std::set<std::string> visited;
        std::string key = endpointKey(
            {SuspensionEndpointKind::Attachment, attachment.id});
        std::size_t hangCount = 0;
        while (true) {
            if (!visited.insert(key).second) {
                fail(SuspensionPhase::Validation, attachment.id,
                     "cycle in attachment path");
            }
            const auto next = parent.find(key);
            if (next == parent.end()) {
                if (key.rfind(std::to_string(enumValue(
                                  SuspensionEndpointKind::HangPoint)) + ":", 0) == 0) {
                    ++hangCount;
                }
                break;
            }
            key = next->second;
            if (key.rfind(std::to_string(enumValue(
                              SuspensionEndpointKind::Junction)) + ":", 0) == 0) {
                reachedJunctions.insert(key.substr(key.find(':') + 1));
            }
        }
        if (hangCount != 1) {
            fail(SuspensionPhase::Validation, attachment.id,
                 "path must terminate at exactly one hang point");
        }
    }
    for (const auto& junction : definition.junctions) {
        const std::string key = endpointKey(
            {SuspensionEndpointKind::Junction, junction.id});
        if (!reachedJunctions.contains(junction.id) ||
            !incoming.contains(key) || !parent.contains(key)) {
            fail(SuspensionPhase::Validation, junction.id,
                 "orphan or disconnected junction");
        }
    }

    const auto segmentExists = [&](const std::string& id) {
        return std::ranges::any_of(definition.segments,
            [&](const auto& segment) { return segment.id == id; });
    };
    const auto hangExists = [&](const std::string& id) {
        return std::ranges::any_of(payload.hangPoints,
            [&](const auto& point) { return point.id == id; });
    };
    std::set<std::pair<int, std::string>> mappedTargets;
    for (const auto& control : definition.controls) {
        if (!validControlKind(control.kind) || !validSide(control.side) ||
            !finite(control.minimumCommand) ||
            !finite(control.maximumCommand) ||
            !finite(control.neutralCommand) || !finite(control.maximumRate) ||
            !(control.minimumCommand < control.maximumCommand) ||
            control.neutralCommand < control.minimumCommand ||
            control.neutralCommand > control.maximumCommand ||
            !(control.maximumRate > 0.0) || control.targets.empty() ||
            control.targets.size() > kMaximumRecords) {
            fail(SuspensionPhase::Validation, control.id,
                 "invalid control bounds, neutral, rate, side, or targets");
        }
        static_cast<void>(provenanceById(definition, control.provenanceId,
                                         control.id));
        for (const auto& target : control.targets) {
            requireText(target.entityId, SuspensionPhase::Validation,
                        control.id);
            if (!validControlTargetKind(target.kind) ||
                !finite(target.travelPerCommand) ||
                !finite(target.localDirection)) {
                fail(SuspensionPhase::Validation, control.id,
                     "non-finite control target");
            }
            if (target.kind ==
                    SuspensionControlTargetKind::SegmentRestLength &&
                !segmentExists(target.entityId)) {
                fail(SuspensionPhase::Validation, control.id,
                     "foreign segment control target");
            }
            if (target.kind ==
                    SuspensionControlTargetKind::HangPointTravel &&
                (!hangExists(target.entityId) ||
                 !(length(target.localDirection) > 0.0))) {
                fail(SuspensionPhase::Validation, control.id,
                     "foreign or directionless hang-point target");
            }
            const auto key = std::pair{enumValue(target.kind), target.entityId};
            if (!mappedTargets.insert(key).second) {
                fail(SuspensionPhase::Validation, control.id,
                     "duplicate control mapping");
            }
        }
    }

    if (definition.solver.lineIterations <= 0 ||
        !(definition.solver.attachmentTolerance > 0.0) ||
        !finite(definition.solver.attachmentTolerance) ||
        !(definition.solver.minimumLineLength > 0.0) ||
        !finite(definition.solver.minimumLineLength) ||
        !(definition.solver.maximumLineResidual >= 0.0) ||
        !finite(definition.solver.maximumLineResidual) ||
        !(definition.solver.maximumControlWork >= 0.0) ||
        !finite(definition.solver.maximumControlWork)) {
        fail(SuspensionPhase::Validation, "solver",
             "invalid coupled solver settings");
    }
    const double normalLength = length(definition.ground.planeNormal);
    if (!validGroundMode(definition.ground.mode) ||
        !finite(definition.ground.planeNormal) ||
        !finite(definition.ground.planeOffset) ||
        definition.ground.compliance < 0.0 ||
        !finite(definition.ground.compliance) ||
        !(definition.ground.penetrationFraction > 0.0) ||
        !finite(definition.ground.penetrationFraction) ||
        (definition.ground.mode == PayloadGroundMode::SupportPlane &&
         (std::abs(normalLength - 1.0) > 1.0e-12 ||
          payload.supportPoints.empty()))) {
        fail(SuspensionPhase::Validation, "ground",
             "invalid ground mode, unit plane, compliance, or support points");
    }

    const auto byId = [](const auto& first, const auto& second) {
        return first.id < second.id;
    };
    std::ranges::sort(definition.provenance, byId);
    std::ranges::sort(definition.attachments, byId);
    std::ranges::sort(definition.junctions, byId);
    std::ranges::sort(definition.payload.hangPoints, byId);
    std::ranges::sort(definition.payload.supportPoints, byId);
    std::ranges::sort(definition.segments, byId);
    std::ranges::sort(definition.controls, byId);
    for (auto& segment : definition.segments)
        std::ranges::sort(segment.groups);
    for (auto& control : definition.controls) {
        std::ranges::sort(control.targets, [](const auto& first,
                                              const auto& second) {
            return std::pair{enumValue(first.kind), first.entityId} <
                   std::pair{enumValue(second.kind), second.entityId};
        });
    }
    return definition;
}

Vec3 payloadPointPosition(const RigidPayloadDefinition& definition,
                          const RigidPayloadState& state,
                          const Vec3& localPoint) {
    return state.centreOfMassWorld +
           rotate(state.orientation,
                  localPoint - definition.centreOfMassLocal);
}

Vec3 payloadPointVelocity(const RigidPayloadDefinition& definition,
                          const RigidPayloadState& state,
                          const Vec3& localPoint) {
    const Vec3 offset = rotate(state.orientation,
                               localPoint - definition.centreOfMassLocal);
    return state.linearVelocity + cross(state.angularVelocity, offset);
}

Mat3 payloadWorldInertia(const RigidPayloadDefinition& definition,
                         const RigidPayloadState& state) {
    const Mat3 rotation = rotationMatrix(state.orientation);
    return rotation * definition.inertiaBody * transpose(rotation);
}

Vec3 payloadLinearMomentum(const RigidPayloadDefinition& definition,
                           const RigidPayloadState& state) {
    return definition.mass * state.linearVelocity;
}

Vec3 payloadAngularMomentum(const RigidPayloadDefinition& definition,
                            const RigidPayloadState& state) {
    return payloadWorldInertia(definition, state) * state.angularVelocity;
}

double payloadKineticEnergy(const RigidPayloadDefinition& definition,
                            const RigidPayloadState& state) {
    return 0.5 * definition.mass * lengthSquared(state.linearVelocity) +
           0.5 * dot(state.angularVelocity,
                     payloadAngularMomentum(definition, state));
}

void applyPayloadImpulse(const RigidPayloadDefinition& definition,
                         RigidPayloadState& state,
                         const Vec3& worldPoint,
                         const Vec3& impulse) {
    if (!finite(worldPoint) || !finite(impulse)) {
        fail(SuspensionPhase::Prediction, "payload-impulse",
             "non-finite payload impulse");
    }
    state.linearVelocity += impulse / definition.mass;
    const Vec3 offset = worldPoint - state.centreOfMassWorld;
    state.angularVelocity +=
        inverse(payloadWorldInertia(definition, state)) * cross(offset, impulse);
    if (!finite(state.linearVelocity) || !finite(state.angularVelocity)) {
        fail(SuspensionPhase::Prediction, "payload-impulse",
             "payload impulse produced non-finite state");
    }
}

void integratePayloadTorqueFree(const RigidPayloadDefinition& definition,
                                RigidPayloadState& state,
                                double timeStep,
                                const Vec3& force,
                                const Vec3& moment) {
    if (!(timeStep > 0.0) || !finite(timeStep) || !finite(force) ||
        !finite(moment)) {
        fail(SuspensionPhase::Prediction, "payload-step",
             "invalid payload integration input");
    }
    state.orientation = normalizedCanonical(state.orientation);
    const Mat3 inertia = payloadWorldInertia(definition, state);
    Vec3 angularMomentum = inertia * state.angularVelocity;
    state.linearVelocity += force * (timeStep / definition.mass);
    angularMomentum += moment * timeStep;
    state.angularVelocity = inverse(inertia) * angularMomentum;
    state.centreOfMassWorld += state.linearVelocity * timeStep;
    state.orientation = normalizedCanonical(
        rotationIncrement(state.angularVelocity * timeStep) *
        state.orientation);
    state.angularVelocity =
        inverse(payloadWorldInertia(definition, state)) * angularMomentum;
    if (!finite(state.centreOfMassWorld) || !finite(state.linearVelocity) ||
        !finite(state.angularVelocity) || !finite(state.orientation)) {
        fail(SuspensionPhase::Prediction, "payload-step",
             "payload integration produced non-finite state");
    }
}

double generalizedLineEffectiveInverseMass(
    const GeneralizedLineEndpoint& endpoint,
    const Vec3& gradient) {
    if (!(endpoint.inverseMass >= 0.0) || !finite(endpoint.inverseMass) ||
        !finite(gradient) || !finite(endpoint.offset) ||
        !finite(endpoint.inverseWorldInertia)) {
        fail(SuspensionPhase::LineSolve, "effective-mass",
             "non-finite generalized endpoint");
    }
    double result = endpoint.inverseMass;
    if (endpoint.rigid && endpoint.inverseMass > 0.0) {
        const Vec3 rotational = cross(endpoint.offset, gradient);
        result += dot(rotational,
                      endpoint.inverseWorldInertia * rotational);
    }
    if (!(result >= 0.0) || !finite(result))
        fail(SuspensionPhase::LineSolve, "effective-mass",
             "invalid generalized effective inverse mass");
    return result;
}

UnilateralLineProjection projectUnilateralLine(
    double lengthValue,
    double commandedRestLength,
    double stiffness,
    double timeStep,
    double accumulatedMultiplier,
    const GeneralizedLineEndpoint& first,
    const GeneralizedLineEndpoint& second,
    const Vec3& firstGradient) {
    if (!(lengthValue > 0.0) || !finite(lengthValue) ||
        !(commandedRestLength > 0.0) || !finite(commandedRestLength) ||
        !(stiffness > 0.0) || !finite(stiffness) ||
        !(timeStep > 0.0) || !finite(timeStep) ||
        !(accumulatedMultiplier >= 0.0) ||
        !finite(accumulatedMultiplier) ||
        std::abs(length(firstGradient) - 1.0) > 1.0e-10) {
        fail(SuspensionPhase::LineSolve, "line-projection",
             "invalid unilateral line projection input");
    }
    UnilateralLineProjection result;
    result.effectiveInverseMass =
        generalizedLineEffectiveInverseMass(first, firstGradient) +
        generalizedLineEffectiveInverseMass(second, -firstGradient);
    const double alphaTilde = 1.0 / (stiffness * timeStep * timeStep);
    if (!(result.effectiveInverseMass + alphaTilde > 0.0))
        fail(SuspensionPhase::LineSolve, "line-projection",
             "zero generalized effective mass");
    const double gap = commandedRestLength - lengthValue;
    const double candidate = accumulatedMultiplier +
        (-gap - alphaTilde * accumulatedMultiplier) /
            (result.effectiveInverseMass + alphaTilde);
    result.accumulatedMultiplier = std::max(0.0, candidate);
    result.multiplierIncrement = result.accumulatedMultiplier -
                                 accumulatedMultiplier;
    result.tension = result.accumulatedMultiplier /
                     (timeStep * timeStep);
    result.taut = result.accumulatedMultiplier > 0.0;
    return result;
}

double boundedAxialDampingImpulse(double relativeAxialVelocity,
                                  double damping,
                                  double timeStep,
                                  double effectiveInverseMass) {
    if (!finite(relativeAxialVelocity) || !(damping >= 0.0) ||
        !finite(damping) || !(timeStep > 0.0) || !finite(timeStep) ||
        !(effectiveInverseMass >= 0.0) ||
        !finite(effectiveInverseMass)) {
        fail(SuspensionPhase::LineSolve, "line-damping",
             "invalid axial damping input");
    }
    if (damping == 0.0 || effectiveInverseMass == 0.0 ||
        relativeAxialVelocity == 0.0)
        return 0.0;
    const double requested = damping * std::abs(relativeAxialVelocity) *
                             timeStep;
    const double stopping = std::abs(relativeAxialVelocity) /
                            effectiveInverseMass;
    return std::copysign(std::min(requested, stopping),
                         relativeAxialVelocity);
}

namespace {

std::size_t indexById(std::span<const ResolvedSuspensionAttachment> values,
                      const std::string& id) {
    const auto found = std::ranges::find_if(
        values, [&](const auto& value) { return value.id == id; });
    if (found == values.end())
        fail(SuspensionPhase::GraphConstruction, id,
             "resolved attachment not found");
    return static_cast<std::size_t>(found - values.begin());
}

std::size_t payloadPointIndex(const RigidPayloadDefinition& payload,
                              const std::string& id) {
    const auto found = std::ranges::find_if(
        payload.hangPoints,
        [&](const PayloadPointDefinition& value) { return value.id == id; });
    if (found == payload.hangPoints.end())
        fail(SuspensionPhase::GraphConstruction, id,
             "payload hang point not found");
    return static_cast<std::size_t>(found - payload.hangPoints.begin());
}

RuntimeSuspensionEndpoint resolveRuntimeEndpoint(
    const SuspensionDefinition& definition,
    std::span<const ResolvedSuspensionAttachment> attachments,
    std::span<const std::size_t> junctionNodes,
    const SuspensionEndpoint& endpoint) {
    RuntimeSuspensionEndpoint result;
    result.definition = endpoint;
    if (endpoint.kind == SuspensionEndpointKind::Attachment) {
        const auto index = indexById(attachments, endpoint.id);
        result.nodeIndex = attachments[index].nodeIndex;
    } else if (endpoint.kind == SuspensionEndpointKind::Junction) {
        const auto found = std::ranges::find_if(
            definition.junctions,
            [&](const auto& value) { return value.id == endpoint.id; });
        if (found == definition.junctions.end())
            fail(SuspensionPhase::GraphConstruction, endpoint.id,
                 "junction not found");
        result.nodeIndex = junctionNodes[static_cast<std::size_t>(
            found - definition.junctions.begin())];
    } else if (endpoint.kind == SuspensionEndpointKind::HangPoint) {
        result.payloadPointIndex = payloadPointIndex(definition.payload,
                                                     endpoint.id);
    } else {
        fail(SuspensionPhase::GraphConstruction, endpoint.id,
             "unsupported endpoint kind");
    }
    return result;
}

Vec3 endpointPosition(const SuspensionSystem& system,
                      const SoftBody& body,
                      const RuntimeSuspensionEndpoint& endpoint) {
    if (endpoint.nodeIndex)
        return body.nodes()[*endpoint.nodeIndex].position;
    const std::size_t index = *endpoint.payloadPointIndex;
    return system.hangPointPosition(index);
}

} // namespace

SuspensionSystem SuspensionSystem::build(
    CanopyMesh& canopy,
    const SuspensionDefinition& input) {
    const SuspensionDefinition definition =
        validateAndNormalizeSuspensionDefinition(input);

    std::vector<ResolvedSuspensionAttachment> resolved;
    resolved.reserve(definition.attachments.size());
    for (const auto& attachment : definition.attachments) {
        std::set<std::size_t> matches;
        for (const CanopyFaceRecord& face : canopy.faces) {
            if (face.virtualClosure || face.panelId != attachment.panelId)
                continue;
            const Triangle& triangle = canopy.body.triangles()[face.triangle];
            const std::array<std::size_t, 3> nodes{triangle.a, triangle.b,
                                                   triangle.c};
            for (std::size_t corner = 0; corner < 3; ++corner) {
                const Vec2 delta{face.chart[corner].x - attachment.chart.x,
                                 face.chart[corner].y - attachment.chart.y};
                const double scale = std::max(
                    {1.0, std::abs(attachment.chart.x),
                     std::abs(attachment.chart.y),
                     std::abs(face.chart[corner].x),
                     std::abs(face.chart[corner].y)});
                if (std::sqrt(delta.x * delta.x + delta.y * delta.y) <=
                    definition.solver.attachmentTolerance * scale) {
                    matches.insert(nodes[corner]);
                }
            }
        }
        if (matches.size() != 1) {
            fail(SuspensionPhase::AttachmentResolution, attachment.id,
                 matches.empty()
                     ? "semantic chart location is not an existing material vertex"
                     : "semantic chart location is ambiguous");
        }
        const std::size_t node = *matches.begin();
        resolved.push_back({attachment.id, attachment.panelId,
                            attachment.chart, node,
                            canopy.body.nodes()[node].position,
                            attachment.provenanceId});
    }

    const std::size_t originalNodeCount = canopy.body.nodes().size();
    std::vector<std::size_t> junctionNodes;
    junctionNodes.reserve(definition.junctions.size());
    try {
        for (const auto& junction : definition.junctions) {
            junctionNodes.push_back(canopy.body.addNode(
                junction.initialWorldPosition, junction.mass));
        }
    } catch (...) {
        canopy.body.nodes().resize(originalNodeCount);
        throw;
    }

    SuspensionSystem system;
    system.definition_ = definition;
    system.owner_ = &canopy.body;
    system.attachments_ = std::move(resolved);
    system.junctionNodeIndices_ = std::move(junctionNodes);
    system.payloadState_ = definition.payload.initialState;
    system.previousPayloadState_ = system.payloadState_;
    for (const auto& point : definition.payload.hangPoints) {
        system.baseHangPointPositions_.push_back(point.localPosition);
        system.commandedHangPointPositions_.push_back(point.localPosition);
    }
    for (const auto& control : definition.controls) {
        system.controls_.push_back({control.id, control.neutralCommand,
                                    control.neutralCommand, 0.0, 0.0});
    }
    for (const auto& segment : definition.segments) {
        RuntimeSuspensionSegment runtime;
        runtime.definition = segment;
        runtime.from = resolveRuntimeEndpoint(
            definition, system.attachments_, system.junctionNodeIndices_,
            segment.from);
        runtime.to = resolveRuntimeEndpoint(
            definition, system.attachments_, system.junctionNodeIndices_,
            segment.to);
        runtime.commandedRestLength = segment.restLength;
        for (const auto& attachment : definition.attachments) {
            SuspensionEndpoint endpoint{SuspensionEndpointKind::Attachment,
                                        attachment.id};
            std::set<std::string> visited;
            while (true) {
                if (!visited.insert(endpointKey(endpoint)).second) break;
                const auto found = std::ranges::find_if(
                    definition.segments,
                    [&](const auto& candidate) {
                        return endpointKey(candidate.from) == endpointKey(endpoint);
                    });
                if (found == definition.segments.end()) break;
                if (found->id == segment.id) {
                    runtime.attachmentPaths.push_back(attachment.id);
                    break;
                }
                endpoint = found->to;
            }
        }
        system.segments_.push_back(std::move(runtime));
    }
    system.groundMultipliers_.assign(definition.payload.supportPoints.size(),
                                     0.0);
    system.pendingSegmentControlWork_.assign(system.segments_.size(), 0.0);
    system.segmentDiagnostics_.resize(system.segments_.size());
    system.diagnostics_.registered = true;
    system.diagnostics_.attachmentCount = system.attachments_.size();
    system.diagnostics_.junctionCount = system.junctionNodeIndices_.size();
    system.diagnostics_.segmentCount = system.segments_.size();
    system.diagnostics_.provenance = definition.provenance.front().id;
    system.committedDiagnostics_ = system.diagnostics_;
    return system;
}

SoftBody& SuspensionSystem::owner() const {
    if (owner_ == nullptr)
        fail(SuspensionPhase::Validation, "owner",
             "unregistered suspension has no owner");
    return *owner_;
}

Vec3 SuspensionSystem::hangPointPosition(std::size_t pointIndex) const {
    if (pointIndex >= commandedHangPointPositions_.size())
        throw std::out_of_range("Suspension hang-point index out of range");
    return payloadPointPosition(definition_.payload, payloadState_,
                                commandedHangPointPositions_[pointIndex]);
}

Vec3 SuspensionSystem::hangPointVelocity(std::size_t pointIndex) const {
    if (pointIndex >= commandedHangPointPositions_.size())
        throw std::out_of_range("Suspension hang-point index out of range");
    return payloadPointVelocity(definition_.payload, payloadState_,
                                commandedHangPointPositions_[pointIndex]);
}

void SuspensionSystem::requireOwner(const SoftBody& body) const {
    if (owner_ != &body) {
        fail(SuspensionPhase::Validation, "owner",
             "foreign structural owner");
    }
}

void SuspensionSystem::setControlTarget(std::string_view controlId,
                                        double command) {
    if (!finite(command))
        fail(SuspensionPhase::Control, std::string(controlId),
             "control command is non-finite");
    const auto definitionFound = std::ranges::find_if(
        definition_.controls,
        [&](const auto& control) { return control.id == controlId; });
    const auto stateFound = std::ranges::find_if(
        controls_, [&](const auto& control) { return control.id == controlId; });
    if (definitionFound == definition_.controls.end() ||
        stateFound == controls_.end())
        fail(SuspensionPhase::Control, std::string(controlId),
             "unknown control channel");
    if (command < definitionFound->minimumCommand ||
        command > definitionFound->maximumCommand) {
        if (!definitionFound->clampOutOfRange)
            fail(SuspensionPhase::Control, definitionFound->id,
                 "control command is outside declared bounds");
        command = std::clamp(command, definitionFound->minimumCommand,
                             definitionFound->maximumCommand);
    }
    stateFound->targetCommand = command;
}

void SuspensionSystem::resetControls() {
    for (std::size_t i = 0; i < controls_.size(); ++i) {
        controls_[i] = {definition_.controls[i].id,
                        definition_.controls[i].neutralCommand,
                        definition_.controls[i].neutralCommand, 0.0, 0.0};
    }
    for (std::size_t i = 0; i < segments_.size(); ++i)
        segments_[i].commandedRestLength =
            segments_[i].definition.restLength;
    commandedHangPointPositions_ = baseHangPointPositions_;
}

void SuspensionSystem::setPayloadState(const RigidPayloadState& state) {
    RigidPayloadState checked = state;
    checked.orientation = normalizedCanonical(checked.orientation);
    if (!finite(checked.centreOfMassWorld) ||
        !finite(checked.linearVelocity) || !finite(checked.angularVelocity))
        fail(SuspensionPhase::Validation, "payload-state",
             "payload state is non-finite");
    payloadState_ = checked;
    previousPayloadState_ = checked;
}

void SuspensionSystem::setAppliedPayloadWrench(const Wrench& wrench) {
    if (!finite(wrench.force) || !finite(wrench.moment))
        fail(SuspensionPhase::Validation, "payload-wrench",
             "applied payload wrench is non-finite");
    appliedWrench_ = wrench;
}

void SuspensionSystem::step(const StepSettings& settings) {
    owner().stepCoupled(settings, *this);
}

void SuspensionSystem::beginSubstep(SoftBody& body,
                                    double dt,
                                    const Vec3& gravity) {
    requireOwner(body);
    snapshotPayloadState_ = payloadState_;
    snapshotPreviousPayloadState_ = previousPayloadState_;
    snapshotCommandedHangPointPositions_ = commandedHangPointPositions_;
    snapshotSegments_ = segments_;
    snapshotControls_ = controls_;
    snapshotGroundMultipliers_ = groundMultipliers_;
    snapshotCurrentGravity_ = currentGravity_;
    snapshotPendingSegmentControlWork_ = pendingSegmentControlWork_;
    snapshotSegmentDiagnostics_ = segmentDiagnostics_;
    snapshotPayloadDiagnostics_ = payloadDiagnostics_;
    snapshotDiagnostics_ = diagnostics_;
    hasSnapshot_ = true;
    previousPayloadState_ = payloadState_;
    currentGravity_ = gravity;

    for (auto& segment : segments_) {
        segment.accumulatedLambda = 0.0;
        segment.commandedRestLength = segment.definition.restLength;
    }
    std::ranges::fill(groundMultipliers_, 0.0);
    std::ranges::fill(pendingSegmentControlWork_, 0.0);
    commandedHangPointPositions_ = baseHangPointPositions_;
    double controlWork = 0.0;
    for (std::size_t controlIndex = 0; controlIndex < controls_.size();
         ++controlIndex) {
        SuspensionControlState& state = controls_[controlIndex];
        const SuspensionControlDefinition& definition =
            definition_.controls[controlIndex];
        const double oldActual = state.actualCommand;
        const double maximumChange = definition.maximumRate * dt;
        state.actualCommand += std::clamp(state.targetCommand - oldActual,
                                          -maximumChange, maximumChange);
        state.travel = 0.0;
        for (const auto& target : definition.targets) {
            const double displacement =
                target.travelPerCommand *
                (state.actualCommand - definition.neutralCommand);
            const double increment =
                target.travelPerCommand * (state.actualCommand - oldActual);
            state.travel = std::max(state.travel, std::abs(displacement));
            if (target.kind ==
                SuspensionControlTargetKind::SegmentRestLength) {
                const auto segment = std::ranges::find_if(
                    segments_, [&](const auto& value) {
                        return value.definition.id == target.entityId;
                    });
                if (segment == segments_.end())
                    fail(SuspensionPhase::Control, definition.id,
                         "runtime segment target disappeared");
                segment->commandedRestLength += displacement;
                const double priorTension = segmentDiagnostics_.empty()
                    ? 0.0
                    : segmentDiagnostics_[static_cast<std::size_t>(
                          segment - segments_.begin())].tension;
                const double work = -priorTension * increment;
                state.signedWork += work;
                controlWork += work;
                pendingSegmentControlWork_[static_cast<std::size_t>(
                    segment - segments_.begin())] += work;
            } else {
                const std::size_t point =
                    payloadPointIndex(definition_.payload, target.entityId);
                commandedHangPointPositions_[point] +=
                    normalized(target.localDirection) * displacement;
                const double work = length(appliedWrench_.force) * increment;
                state.signedWork += work;
                controlWork += work;
            }
        }
    }
    for (const auto& segment : segments_) {
        if (!(segment.commandedRestLength >
              definition_.solver.minimumLineLength) ||
            !finite(segment.commandedRestLength)) {
            fail(SuspensionPhase::Control, segment.definition.id,
                 "control produced invalid commanded rest length");
        }
    }
    if (!finite(controlWork) ||
        std::abs(controlWork) > definition_.solver.maximumControlWork) {
        fail(SuspensionPhase::Control, "work-budget",
             "prescribed control work budget exhausted");
    }

    diagnostics_ = {};
    diagnostics_.registered = true;
    diagnostics_.attachmentCount = attachments_.size();
    diagnostics_.junctionCount = junctionNodeIndices_.size();
    diagnostics_.segmentCount = segments_.size();
    diagnostics_.solverIterations = 0;
    diagnostics_.controlWork = controlWork;
    diagnostics_.anchored =
        definition_.ground.mode == PayloadGroundMode::Anchored;
    diagnostics_.provenance = definition_.provenance.front().id;
    segmentDiagnostics_.assign(segments_.size(), {});

    if (definition_.ground.mode == PayloadGroundMode::Anchored) {
        payloadState_ = definition_.payload.initialState;
        payloadState_.linearVelocity = {};
        payloadState_.angularVelocity = {};
    } else {
        const Vec3 totalForce = appliedWrench_.force +
                                definition_.payload.mass * gravity;
        integratePayloadTorqueFree(definition_.payload, payloadState_, dt,
                                   totalForce, appliedWrench_.moment);
    }
}

namespace {

struct EndpointSolveState {
    Vec3 position;
    Vec3 velocity;
    double inverseMass = 0.0;
    Vec3 offset;
    bool rigid = false;
    bool fixed = false;
};

} // namespace

void SuspensionSystem::solveLineIteration(SoftBody& body, double dt) {
    requireOwner(body);
    const Mat3 inverseInertia = inverse(payloadWorldInertia(
        definition_.payload, payloadState_));
    const auto stateFor = [&](const RuntimeSuspensionEndpoint& endpoint) {
        EndpointSolveState state;
        if (endpoint.nodeIndex) {
            const Node& node = body.nodes()[*endpoint.nodeIndex];
            state.position = node.position;
            state.velocity = node.velocity;
            state.inverseMass = node.inverseMass;
            state.fixed = node.inverseMass == 0.0;
        } else {
            const std::size_t point = *endpoint.payloadPointIndex;
            state.position = payloadPointPosition(
                definition_.payload, payloadState_,
                commandedHangPointPositions_[point]);
            state.velocity = payloadPointVelocity(
                definition_.payload, payloadState_,
                commandedHangPointPositions_[point]);
            state.offset = state.position - payloadState_.centreOfMassWorld;
            state.rigid = true;
            state.fixed = definition_.ground.mode ==
                          PayloadGroundMode::Anchored;
            state.inverseMass = state.fixed ? 0.0 : 1.0 / definition_.payload.mass;
        }
        return state;
    };
    const auto applyCorrection = [&](const RuntimeSuspensionEndpoint& endpoint,
                                     const EndpointSolveState& state,
                                     const Vec3& gradient,
                                     double multiplier) {
        if (state.fixed) return;
        if (endpoint.nodeIndex) {
            body.nodes()[*endpoint.nodeIndex].position +=
                state.inverseMass * multiplier * gradient;
        } else {
            payloadState_.centreOfMassWorld +=
                state.inverseMass * multiplier * gradient;
            const Vec3 rotation = inverseInertia *
                                  cross(state.offset, gradient) * multiplier;
            payloadState_.orientation = normalizedCanonical(
                rotationIncrement(rotation) * payloadState_.orientation);
        }
    };

    for (std::size_t index = 0; index < segments_.size(); ++index) {
        RuntimeSuspensionSegment& segment = segments_[index];
        const EndpointSolveState from = stateFor(segment.from);
        const EndpointSolveState to = stateFor(segment.to);
        const Vec3 difference = to.position - from.position;
        const double currentLength = length(difference);
        if (!finite(currentLength) ||
            currentLength <= definition_.solver.minimumLineLength) {
            fail(SuspensionPhase::LineSolve, segment.definition.id,
                 "line geometry is non-finite or scale-degenerate");
        }
        const Vec3 direction = difference / currentLength;
        const Vec3 fromGradient = direction;
        const Vec3 toGradient = -direction;
        const GeneralizedLineEndpoint fromEndpoint{
            from.inverseMass, from.rigid && !from.fixed, from.offset,
            inverseInertia};
        const GeneralizedLineEndpoint toEndpoint{
            to.inverseMass, to.rigid && !to.fixed, to.offset,
            inverseInertia};
        const UnilateralLineProjection projection = projectUnilateralLine(
            currentLength, segment.commandedRestLength,
            segment.definition.axialStiffness, dt,
            segment.accumulatedLambda, fromEndpoint, toEndpoint,
            fromGradient);
        const double increment = projection.multiplierIncrement;
        segment.accumulatedLambda = projection.accumulatedMultiplier;
        applyCorrection(segment.from, from, fromGradient, increment);
        applyCorrection(segment.to, to, toGradient, increment);
        diagnostics_.solverIterations += 1;
    }
}

void SuspensionSystem::solveGroundIteration(double dt) {
    if (definition_.ground.mode == PayloadGroundMode::Free) return;
    if (definition_.ground.mode == PayloadGroundMode::Anchored) {
        payloadState_ = definition_.payload.initialState;
        payloadState_.linearVelocity = {};
        payloadState_.angularVelocity = {};
        return;
    }
    const Mat3 inverseInertia = inverse(payloadWorldInertia(
        definition_.payload, payloadState_));
    const Vec3 normal = definition_.ground.planeNormal;
    for (std::size_t index = 0;
         index < definition_.payload.supportPoints.size(); ++index) {
        const Vec3 point = payloadPointPosition(
            definition_.payload, payloadState_,
            definition_.payload.supportPoints[index].localPosition);
        const Vec3 offset = point - payloadState_.centreOfMassWorld;
        const double gap = dot(normal, point) - definition_.ground.planeOffset;
        const Vec3 rotational = cross(offset, normal);
        const double effective = 1.0 / definition_.payload.mass +
            dot(rotational, inverseInertia * rotational);
        const double alphaTilde = definition_.ground.compliance / (dt * dt);
        const double candidate = groundMultipliers_[index] +
            (-gap - alphaTilde * groundMultipliers_[index]) /
                (effective + alphaTilde);
        if (!finite(point) || !finite(offset) || !finite(gap) ||
            !finite(rotational) || !(effective > 0.0) || !finite(effective) ||
            !finite(alphaTilde) || !finite(candidate)) {
            fail(SuspensionPhase::GroundSolve,
                 definition_.payload.supportPoints[index].id,
                 "support-plane solve produced a non-finite state");
        }
        const double updated = std::max(0.0, candidate);
        const double increment = updated - groundMultipliers_[index];
        if (!finite(updated) || !finite(increment)) {
            fail(SuspensionPhase::GroundSolve,
                 definition_.payload.supportPoints[index].id,
                 "support-plane multiplier is non-finite");
        }
        groundMultipliers_[index] = updated;
        payloadState_.centreOfMassWorld +=
            normal * (increment / definition_.payload.mass);
        payloadState_.orientation = normalizedCanonical(
            rotationIncrement(inverseInertia * rotational * increment) *
            payloadState_.orientation);
    }
}

void SuspensionSystem::certifySubstep(const SoftBody& body, double dt) {
    if (!finite(payloadState_.centreOfMassWorld) ||
        !finite(payloadState_.orientation) ||
        !finite(payloadState_.linearVelocity) ||
        !finite(payloadState_.angularVelocity) ||
        std::abs(quaternionNormSquared(payloadState_.orientation) - 1.0) >
            1.0e-12) {
        fail(SuspensionPhase::Certification, "payload",
             "payload trial state is non-finite or unnormalized");
    }
    double maximumResidual = 0.0;
    for (const auto& segment : segments_) {
        const Vec3 from = endpointPosition(*this, body, segment.from);
        const Vec3 to = endpointPosition(*this, body, segment.to);
        const double currentLength = length(to - from);
        if (!finite(currentLength))
            fail(SuspensionPhase::Certification, segment.definition.id,
                 "line trial length is non-finite");
        const double stretch =
            std::max(0.0, currentLength - segment.commandedRestLength);
        const double compliantStretch = segment.accumulatedLambda /
            (segment.definition.axialStiffness * dt * dt);
        maximumResidual = std::max(
            maximumResidual, std::abs(stretch - compliantStretch));
    }
    diagnostics_.maximumResidual = maximumResidual;
    if (maximumResidual > definition_.solver.maximumLineResidual) {
        fail(SuspensionPhase::Certification, "line-residual",
             "maximum line residual exceeds configured bound");
    }
    if (definition_.ground.mode == PayloadGroundMode::SupportPlane) {
        double maximumPenetration = 0.0;
        double supportScale = 0.0;
        for (const auto& point : definition_.payload.supportPoints) {
            const Vec3 world = payloadPointPosition(definition_.payload,
                                                    payloadState_,
                                                    point.localPosition);
            maximumPenetration = std::max(
                maximumPenetration,
                std::max(0.0, definition_.ground.planeOffset -
                                  dot(definition_.ground.planeNormal, world)));
            supportScale = std::max(supportScale,
                length(point.localPosition -
                       definition_.payload.centreOfMassLocal));
        }
        const double limit = std::max(
            1.0e-6,
            definition_.ground.penetrationFraction *
                std::max(1.0e-6, supportScale));
        diagnostics_.maximumGroundPenetration = maximumPenetration;
        if (maximumPenetration > limit)
            fail(SuspensionPhase::Certification, "ground-penetration",
                 "payload support penetration exceeds configured gate");
    }
}

void SuspensionSystem::reconstructPayloadVelocity(double dt) {
    if (definition_.ground.mode == PayloadGroundMode::Anchored) {
        payloadState_ = definition_.payload.initialState;
        payloadState_.linearVelocity = {};
        payloadState_.angularVelocity = {};
        return;
    }
    payloadState_.linearVelocity =
        (payloadState_.centreOfMassWorld -
         previousPayloadState_.centreOfMassWorld) / dt;
    Quaternion delta = payloadState_.orientation *
        Quaternion{previousPayloadState_.orientation.w,
                   -previousPayloadState_.orientation.x,
                   -previousPayloadState_.orientation.y,
                   -previousPayloadState_.orientation.z};
    delta = normalizedCanonical(delta);
    const double halfAngle = std::acos(std::clamp(delta.w, -1.0, 1.0));
    const double sine = std::sin(halfAngle);
    if (std::abs(sine) < 1.0e-12) {
        payloadState_.angularVelocity =
            Vec3{delta.x, delta.y, delta.z} * (2.0 / dt);
    } else {
        const Vec3 axis{delta.x / sine, delta.y / sine, delta.z / sine};
        payloadState_.angularVelocity = axis * (2.0 * halfAngle / dt);
    }
}

void SuspensionSystem::applyLineDamping(SoftBody& body, double dt) {
    const Mat3 inverseInertia = inverse(payloadWorldInertia(
        definition_.payload, payloadState_));
    const auto pointVelocity = [&](const RuntimeSuspensionEndpoint& endpoint) {
        if (endpoint.nodeIndex)
            return body.nodes()[*endpoint.nodeIndex].velocity;
        return payloadPointVelocity(
            definition_.payload, payloadState_,
            commandedHangPointPositions_[*endpoint.payloadPointIndex]);
    };
    const auto apply = [&](const RuntimeSuspensionEndpoint& endpoint,
                           const Vec3& point,
                           const Vec3& impulse) {
        if (endpoint.nodeIndex) {
            Node& node = body.nodes()[*endpoint.nodeIndex];
            if (node.inverseMass > 0.0)
                node.velocity += impulse * node.inverseMass;
        } else if (definition_.ground.mode != PayloadGroundMode::Anchored) {
            payloadState_.linearVelocity += impulse / definition_.payload.mass;
            payloadState_.angularVelocity += inverseInertia *
                cross(point - payloadState_.centreOfMassWorld, impulse);
        }
    };
    for (std::size_t index = 0; index < segments_.size(); ++index) {
        RuntimeSuspensionSegment& segment = segments_[index];
        if (!(segment.accumulatedLambda > 0.0) ||
            !(segment.definition.axialDamping > 0.0))
            continue;
        const Vec3 from = endpointPosition(*this, body, segment.from);
        const Vec3 to = endpointPosition(*this, body, segment.to);
        const Vec3 direction = normalized(to - from);
        const double relative = dot(pointVelocity(segment.to) -
                                    pointVelocity(segment.from), direction);
        double effective = 0.0;
        if (segment.from.nodeIndex)
            effective += body.nodes()[*segment.from.nodeIndex].inverseMass;
        else if (definition_.ground.mode != PayloadGroundMode::Anchored) {
            const Vec3 r = from - payloadState_.centreOfMassWorld;
            const Vec3 rxn = cross(r, direction);
            effective += 1.0 / definition_.payload.mass +
                         dot(rxn, inverseInertia * rxn);
        }
        if (segment.to.nodeIndex)
            effective += body.nodes()[*segment.to.nodeIndex].inverseMass;
        else if (definition_.ground.mode != PayloadGroundMode::Anchored) {
            const Vec3 r = to - payloadState_.centreOfMassWorld;
            const Vec3 rxn = cross(r, direction);
            effective += 1.0 / definition_.payload.mass +
                         dot(rxn, inverseInertia * rxn);
        }
        if (!(effective > 0.0) || relative == 0.0) continue;
        const double signedMagnitude = boundedAxialDampingImpulse(
            relative, segment.definition.axialDamping, dt, effective);
        const double magnitude = std::abs(signedMagnitude);
        const Vec3 fromImpulse = direction * signedMagnitude;
        const Vec3 toImpulse = -fromImpulse;
        const double before = relative;
        apply(segment.from, from, fromImpulse);
        apply(segment.to, to, toImpulse);
        const double work = -magnitude * std::abs(before);
        SuspensionSegmentDiagnostics& diagnostics =
            segmentDiagnostics_[index];
        diagnostics.dampingImpulse += fromImpulse;
        diagnostics.dampingWork += work;
        diagnostics_.dampingWork += work;
    }
}

void SuspensionSystem::finishSubstep(const SoftBody& body, double dt) {
    diagnostics_.tautCount = 0;
    diagnostics_.slackCount = 0;
    diagnostics_.elasticEnergy = 0.0;
    diagnostics_.netInternalImpulse = {};
    diagnostics_.netInternalMoment = {};
    diagnostics_.attachmentLoads.clear();
    diagnostics_.groupLoads.clear();
    payloadDiagnostics_ = {};
    payloadDiagnostics_.state = payloadState_;
    payloadDiagnostics_.linearMomentum =
        payloadLinearMomentum(definition_.payload, payloadState_);
    payloadDiagnostics_.angularMomentum =
        payloadAngularMomentum(definition_.payload, payloadState_);
    payloadDiagnostics_.translationalKineticEnergy =
        0.5 * definition_.payload.mass *
        lengthSquared(payloadState_.linearVelocity);
    payloadDiagnostics_.rotationalKineticEnergy =
        0.5 * dot(payloadState_.angularVelocity,
                  payloadDiagnostics_.angularMomentum);
    payloadDiagnostics_.gravitationalEnergy =
        -definition_.payload.mass *
        dot(currentGravity_, payloadState_.centreOfMassWorld);
    payloadDiagnostics_.appliedWrench = appliedWrench_;

    std::map<std::string, Vec3> attachmentLoads;
    std::map<std::string, Vec3> groupLoads;
    for (std::size_t index = 0; index < segments_.size(); ++index) {
        const RuntimeSuspensionSegment& segment = segments_[index];
        SuspensionSegmentDiagnostics& record = segmentDiagnostics_[index];
        record.id = segment.definition.id;
        record.from = segment.definition.from;
        record.to = segment.definition.to;
        record.paths = segment.attachmentPaths;
        record.groups = segment.definition.groups;
        const Vec3 from = endpointPosition(*this, body, segment.from);
        const Vec3 to = endpointPosition(*this, body, segment.to);
        const Vec3 direction = normalized(to - from);
        record.length = length(to - from);
        record.commandedRestLength = segment.commandedRestLength;
        record.stretch = std::max(0.0,
                                  record.length - record.commandedRestLength);
        record.strain = record.stretch / record.commandedRestLength;
        record.multiplier = segment.accumulatedLambda;
        record.tension = record.multiplier / (dt * dt);
        record.taut = record.multiplier > 0.0;
        record.residual = std::abs(
            record.stretch - record.multiplier /
                (segment.definition.axialStiffness * dt * dt));
        record.elasticEnergy = 0.5 * segment.definition.axialStiffness *
                               record.stretch * record.stretch;
        record.controlWork = pendingSegmentControlWork_[index];
        record.fromImpulse = direction * (record.multiplier / dt);
        record.toImpulse = -record.fromImpulse;
        record.fromMoment = cross(from, record.fromImpulse);
        record.toMoment = cross(to, record.toImpulse);
        diagnostics_.netInternalImpulse +=
            record.fromImpulse + record.toImpulse;
        diagnostics_.netInternalMoment +=
            record.fromMoment + record.toMoment;
        diagnostics_.elasticEnergy += record.elasticEnergy;
        if (record.taut) ++diagnostics_.tautCount;
        else ++diagnostics_.slackCount;
        if (segment.definition.from.kind ==
            SuspensionEndpointKind::Attachment) {
            attachmentLoads[segment.definition.from.id] +=
                direction * record.tension;
            if (segment.from.nodeIndex &&
                body.nodes()[*segment.from.nodeIndex].inverseMass == 0.0) {
                const Vec3 reaction = -direction * record.tension;
                diagnostics_.canopySupportReaction.force += reaction;
                diagnostics_.canopySupportReaction.moment +=
                    cross(from, reaction);
            }
        }
        for (const auto& group : segment.definition.groups)
            groupLoads[group] += direction * record.tension;
        if (segment.from.payloadPointIndex || segment.to.payloadPointIndex) {
            const Vec3 impulse = segment.to.payloadPointIndex
                ? record.toImpulse : record.fromImpulse;
            const Vec3 point = segment.to.payloadPointIndex ? to : from;
            payloadDiagnostics_.lineWrench.force += impulse / dt;
            payloadDiagnostics_.lineWrench.moment +=
                cross(point - payloadState_.centreOfMassWorld,
                      impulse / dt);
        }
    }
    diagnostics_.allSlack = diagnostics_.tautCount == 0;
    for (const auto& [id, load] : attachmentLoads)
        diagnostics_.attachmentLoads.emplace_back(id, load);
    for (const auto& [id, load] : groupLoads)
        diagnostics_.groupLoads.emplace_back(id, load);
    if (definition_.ground.mode == PayloadGroundMode::SupportPlane) {
        for (std::size_t index = 0; index < groundMultipliers_.size(); ++index) {
            const Vec3 force = definition_.ground.planeNormal *
                               (groundMultipliers_[index] / (dt * dt));
            const Vec3 point = payloadPointPosition(
                definition_.payload, payloadState_,
                definition_.payload.supportPoints[index].localPosition);
            diagnostics_.groundReaction.force += force;
            diagnostics_.groundReaction.moment +=
                cross(point - payloadState_.centreOfMassWorld,
                      force);
            diagnostics_.grounded = diagnostics_.grounded ||
                                    groundMultipliers_[index] > 0.0;
        }
        payloadDiagnostics_.groundWrench = diagnostics_.groundReaction;
    }
    if (definition_.ground.mode == PayloadGroundMode::Anchored) {
        const Vec3 gravityForce = definition_.payload.mass * currentGravity_;
        payloadDiagnostics_.anchorWrench.force =
            -(appliedWrench_.force + gravityForce +
              payloadDiagnostics_.lineWrench.force);
        payloadDiagnostics_.anchorWrench.moment =
            -(appliedWrench_.moment + payloadDiagnostics_.lineWrench.moment);
    }
    diagnostics_.fixedSupportReaction = diagnostics_.canopySupportReaction;
    diagnostics_.fixedSupportReaction.force +=
        payloadDiagnostics_.anchorWrench.force;
    diagnostics_.fixedSupportReaction.moment +=
        payloadDiagnostics_.anchorWrench.moment +
        cross(payloadState_.centreOfMassWorld,
              payloadDiagnostics_.anchorWrench.force);
    diagnostics_.converged =
        diagnostics_.maximumResidual <=
        definition_.solver.maximumLineResidual;
    diagnostics_.failedTrial = false;
    committedDiagnostics_ = diagnostics_;
    hasSnapshot_ = false;
}

void SuspensionSystem::rollbackSubstep() noexcept {
    if (!hasSnapshot_) return;
    payloadState_ = snapshotPayloadState_;
    previousPayloadState_ = snapshotPreviousPayloadState_;
    commandedHangPointPositions_ = snapshotCommandedHangPointPositions_;
    segments_ = snapshotSegments_;
    controls_ = snapshotControls_;
    groundMultipliers_ = snapshotGroundMultipliers_;
    currentGravity_ = snapshotCurrentGravity_;
    pendingSegmentControlWork_ = snapshotPendingSegmentControlWork_;
    segmentDiagnostics_ = snapshotSegmentDiagnostics_;
    payloadDiagnostics_ = snapshotPayloadDiagnostics_;
    diagnostics_ = snapshotDiagnostics_;
    diagnostics_.registered = true;
    hasSnapshot_ = false;
}

void SuspensionSystem::recordFailure(SuspensionPhase phase,
                                     const std::string& entity) noexcept {
    diagnostics_.registered = true;
    diagnostics_.failedTrial = true;
    diagnostics_.converged = false;
    diagnostics_.failurePhase = phase;
    diagnostics_.failureEntity = entity;
}

} // namespace softwing

namespace softwing {

std::string serializeSuspensionDefinition(
    const SuspensionDefinition& input) {
    const SuspensionDefinition definition =
        validateAndNormalizeSuspensionDefinition(input);
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << std::scientific;
    output << "SOFTWING_SUSPENSION 1\n";
    output << "IDENTIFIER " << std::quoted(definition.identifier) << '\n';
    output << "DESCRIPTION " << std::quoted(definition.description) << '\n';
    output << "FRAME " << std::quoted(definition.unitsFrameTag) << '\n';
    output << "PROVENANCE " << definition.provenance.size() << '\n';
    for (const auto& record : definition.provenance)
        output << "PROV " << std::quoted(record.id) << ' '
               << std::quoted(record.source) << '\n';
    output << "ATTACHMENTS " << definition.attachments.size() << '\n';
    for (const auto& attachment : definition.attachments)
        output << "ATTACH " << std::quoted(attachment.id) << ' '
               << std::quoted(attachment.panelId) << ' '
               << attachment.chart.x << ' ' << attachment.chart.y << ' '
               << suspensionSideName(attachment.side) << ' '
               << std::quoted(attachment.seamId) << ' '
               << std::quoted(attachment.provenanceId) << '\n';
    output << "JUNCTIONS " << definition.junctions.size() << '\n';
    for (const auto& junction : definition.junctions)
        output << "JUNCTION " << std::quoted(junction.id) << ' '
               << junction.initialWorldPosition.x << ' '
               << junction.initialWorldPosition.y << ' '
               << junction.initialWorldPosition.z << ' ' << junction.mass << ' '
               << suspensionSideName(junction.side) << ' '
               << std::quoted(junction.provenanceId) << '\n';
    const auto& payload = definition.payload;
    output << "PAYLOAD " << payload.mass << ' '
           << payload.centreOfMassLocal.x << ' '
           << payload.centreOfMassLocal.y << ' '
           << payload.centreOfMassLocal.z;
    for (const double value : payload.inertiaBody.values) output << ' ' << value;
    output << ' ' << payload.initialState.centreOfMassWorld.x << ' '
           << payload.initialState.centreOfMassWorld.y << ' '
           << payload.initialState.centreOfMassWorld.z << ' '
           << payload.initialState.orientation.w << ' '
           << payload.initialState.orientation.x << ' '
           << payload.initialState.orientation.y << ' '
           << payload.initialState.orientation.z << ' '
           << payload.initialState.linearVelocity.x << ' '
           << payload.initialState.linearVelocity.y << ' '
           << payload.initialState.linearVelocity.z << ' '
           << payload.initialState.angularVelocity.x << ' '
           << payload.initialState.angularVelocity.y << ' '
           << payload.initialState.angularVelocity.z << ' '
           << std::quoted(payload.provenanceId) << '\n';
    output << "HANG_POINTS " << payload.hangPoints.size() << '\n';
    for (const auto& point : payload.hangPoints)
        output << "HANG " << std::quoted(point.id) << ' '
               << point.localPosition.x << ' ' << point.localPosition.y << ' '
               << point.localPosition.z << ' '
               << suspensionSideName(point.side) << ' '
               << std::quoted(point.provenanceId) << '\n';
    output << "SUPPORT_POINTS " << payload.supportPoints.size() << '\n';
    for (const auto& point : payload.supportPoints)
        output << "SUPPORT " << std::quoted(point.id) << ' '
               << point.localPosition.x << ' ' << point.localPosition.y << ' '
               << point.localPosition.z << ' '
               << suspensionSideName(point.side) << ' '
               << std::quoted(point.provenanceId) << '\n';
    output << "SEGMENTS " << definition.segments.size() << '\n';
    for (const auto& segment : definition.segments) {
        output << "SEGMENT " << std::quoted(segment.id) << ' '
               << suspensionEndpointKindName(segment.from.kind) << ' '
               << std::quoted(segment.from.id) << ' '
               << suspensionEndpointKindName(segment.to.kind) << ' '
               << std::quoted(segment.to.id) << ' ' << segment.restLength << ' '
               << segment.axialStiffness << ' ' << segment.axialDamping << ' '
               << suspensionSideName(segment.side) << ' '
               << segment.groups.size();
        for (const auto& group : segment.groups)
            output << ' ' << std::quoted(group);
        output << ' ' << std::quoted(segment.provenanceId) << '\n';
    }
    output << "CONTROLS " << definition.controls.size() << '\n';
    for (const auto& control : definition.controls) {
        output << "CONTROL " << std::quoted(control.id) << ' '
               << suspensionControlKindName(control.kind) << ' '
               << suspensionSideName(control.side) << ' '
               << control.minimumCommand << ' ' << control.maximumCommand << ' '
               << control.neutralCommand << ' ' << control.maximumRate << ' '
               << (control.clampOutOfRange ? 1 : 0) << ' '
               << control.targets.size() << ' '
               << std::quoted(control.provenanceId) << '\n';
        for (const auto& target : control.targets)
            output << "TARGET " << controlTargetKindName(target.kind) << ' '
                   << std::quoted(target.entityId) << ' '
                   << target.travelPerCommand << ' '
                   << target.localDirection.x << ' '
                   << target.localDirection.y << ' '
                   << target.localDirection.z << '\n';
    }
    output << "SOLVER " << definition.solver.lineIterations << ' '
           << definition.solver.attachmentTolerance << ' '
           << definition.solver.minimumLineLength << ' '
           << definition.solver.maximumLineResidual << ' '
           << definition.solver.maximumControlWork << '\n';
    output << "GROUND " << payloadGroundModeName(definition.ground.mode) << ' '
           << definition.ground.planeNormal.x << ' '
           << definition.ground.planeNormal.y << ' '
           << definition.ground.planeNormal.z << ' '
           << definition.ground.planeOffset << ' '
           << definition.ground.compliance << ' '
           << definition.ground.penetrationFraction << '\n';
    output << "END\n";
    if (!output)
        fail(SuspensionPhase::Diagnostics, "serializer",
             "canonical writer failed before publishing output");
    return output.str();
}

SuspensionDefinition parseSuspensionDefinition(std::string_view text) {
    if (text.size() > 16U * 1024U * 1024U)
        fail(SuspensionPhase::Parse, "document-size",
             "canonical input exceeds bounded allocation limit");
    std::istringstream input{std::string(text)};
    input.imbue(std::locale::classic());
    std::string header;
    std::size_t version = 0;
    if (!(input >> header >> version) || header != "SOFTWING_SUSPENSION" ||
        version != 1)
        fail(SuspensionPhase::Parse, "header",
             "unsupported header or version");

    SuspensionDefinition definition;
    definition.schemaMajor = version;
    expectToken(input, "IDENTIFIER", "identifier");
    definition.identifier = readQuoted(input, "identifier");
    expectToken(input, "DESCRIPTION", "description");
    definition.description = readQuoted(input, "description");
    expectToken(input, "FRAME", "frame");
    definition.unitsFrameTag = readQuoted(input, "frame");
    expectToken(input, "PROVENANCE", "provenance-count");
    const std::size_t provenanceCount =
        checkedCount(input, "provenance-count");
    definition.provenance.reserve(provenanceCount);
    for (std::size_t i = 0; i < provenanceCount; ++i) {
        expectToken(input, "PROV", "provenance");
        definition.provenance.push_back(
            {readQuoted(input, "provenance-id"),
             readQuoted(input, "provenance-source")});
    }
    expectToken(input, "ATTACHMENTS", "attachment-count");
    const std::size_t attachmentCount =
        checkedCount(input, "attachment-count");
    definition.attachments.reserve(attachmentCount);
    for (std::size_t i = 0; i < attachmentCount; ++i) {
        expectToken(input, "ATTACH", "attachment");
        SuspensionAttachmentDefinition value;
        value.id = readQuoted(input, "attachment-id");
        value.panelId = readQuoted(input, value.id);
        value.chart = {readDouble(input, value.id), readDouble(input, value.id)};
        std::string side;
        if (!(input >> side)) fail(SuspensionPhase::Parse, value.id,
                                   "missing attachment side");
        value.side = parseSide(side);
        value.seamId = readQuoted(input, value.id);
        value.provenanceId = readQuoted(input, value.id);
        definition.attachments.push_back(std::move(value));
    }
    expectToken(input, "JUNCTIONS", "junction-count");
    const std::size_t junctionCount = checkedCount(input, "junction-count");
    definition.junctions.reserve(junctionCount);
    for (std::size_t i = 0; i < junctionCount; ++i) {
        expectToken(input, "JUNCTION", "junction");
        SuspensionJunctionDefinition value;
        value.id = readQuoted(input, "junction-id");
        value.initialWorldPosition = {readDouble(input, value.id),
                                      readDouble(input, value.id),
                                      readDouble(input, value.id)};
        value.mass = readDouble(input, value.id);
        std::string side;
        if (!(input >> side)) fail(SuspensionPhase::Parse, value.id,
                                   "missing junction side");
        value.side = parseSide(side);
        value.provenanceId = readQuoted(input, value.id);
        definition.junctions.push_back(std::move(value));
    }
    expectToken(input, "PAYLOAD", "payload");
    auto& payload = definition.payload;
    payload.mass = readDouble(input, "payload-mass");
    payload.centreOfMassLocal = {readDouble(input, "payload-com"),
                                 readDouble(input, "payload-com"),
                                 readDouble(input, "payload-com")};
    for (double& value : payload.inertiaBody.values)
        value = readDouble(input, "payload-inertia");
    payload.initialState.centreOfMassWorld = {
        readDouble(input, "payload-position"),
        readDouble(input, "payload-position"),
        readDouble(input, "payload-position")};
    payload.initialState.orientation = {
        readDouble(input, "payload-orientation"),
        readDouble(input, "payload-orientation"),
        readDouble(input, "payload-orientation"),
        readDouble(input, "payload-orientation")};
    payload.initialState.linearVelocity = {
        readDouble(input, "payload-linear-velocity"),
        readDouble(input, "payload-linear-velocity"),
        readDouble(input, "payload-linear-velocity")};
    payload.initialState.angularVelocity = {
        readDouble(input, "payload-angular-velocity"),
        readDouble(input, "payload-angular-velocity"),
        readDouble(input, "payload-angular-velocity")};
    payload.provenanceId = readQuoted(input, "payload-provenance");

    expectToken(input, "HANG_POINTS", "hang-point-count");
    const std::size_t hangCount = checkedCount(input, "hang-point-count");
    payload.hangPoints.reserve(hangCount);
    for (std::size_t i = 0; i < hangCount; ++i) {
        expectToken(input, "HANG", "hang-point");
        PayloadPointDefinition value;
        value.id = readQuoted(input, "hang-point-id");
        value.localPosition = {readDouble(input, value.id),
                               readDouble(input, value.id),
                               readDouble(input, value.id)};
        std::string side;
        if (!(input >> side)) fail(SuspensionPhase::Parse, value.id,
                                   "missing hang-point side");
        value.side = parseSide(side);
        value.provenanceId = readQuoted(input, value.id);
        payload.hangPoints.push_back(std::move(value));
    }
    expectToken(input, "SUPPORT_POINTS", "support-point-count");
    const std::size_t supportCount =
        checkedCount(input, "support-point-count");
    payload.supportPoints.reserve(supportCount);
    for (std::size_t i = 0; i < supportCount; ++i) {
        expectToken(input, "SUPPORT", "support-point");
        PayloadPointDefinition value;
        value.id = readQuoted(input, "support-point-id");
        value.localPosition = {readDouble(input, value.id),
                               readDouble(input, value.id),
                               readDouble(input, value.id)};
        std::string side;
        if (!(input >> side)) fail(SuspensionPhase::Parse, value.id,
                                   "missing support-point side");
        value.side = parseSide(side);
        value.provenanceId = readQuoted(input, value.id);
        payload.supportPoints.push_back(std::move(value));
    }
    expectToken(input, "SEGMENTS", "segment-count");
    const std::size_t segmentCount = checkedCount(input, "segment-count");
    definition.segments.reserve(segmentCount);
    for (std::size_t i = 0; i < segmentCount; ++i) {
        expectToken(input, "SEGMENT", "segment");
        SuspensionSegmentDefinition value;
        value.id = readQuoted(input, "segment-id");
        std::string fromKind;
        if (!(input >> fromKind)) fail(SuspensionPhase::Parse, value.id,
                                       "missing from endpoint kind");
        value.from.kind = parseEndpointKind(fromKind);
        value.from.id = readQuoted(input, value.id);
        std::string toKind;
        if (!(input >> toKind)) fail(SuspensionPhase::Parse, value.id,
                                     "missing to endpoint kind");
        value.to.kind = parseEndpointKind(toKind);
        value.to.id = readQuoted(input, value.id);
        value.restLength = readDouble(input, value.id);
        value.axialStiffness = readDouble(input, value.id);
        value.axialDamping = readDouble(input, value.id);
        std::string side;
        if (!(input >> side)) fail(SuspensionPhase::Parse, value.id,
                                   "missing segment side");
        value.side = parseSide(side);
        const std::size_t groupCount = checkedCount(input, value.id);
        value.groups.reserve(groupCount);
        for (std::size_t group = 0; group < groupCount; ++group)
            value.groups.push_back(readQuoted(input, value.id));
        value.provenanceId = readQuoted(input, value.id);
        definition.segments.push_back(std::move(value));
    }
    expectToken(input, "CONTROLS", "control-count");
    const std::size_t controlCount = checkedCount(input, "control-count");
    definition.controls.reserve(controlCount);
    for (std::size_t i = 0; i < controlCount; ++i) {
        expectToken(input, "CONTROL", "control");
        SuspensionControlDefinition value;
        value.id = readQuoted(input, "control-id");
        std::string kind;
        std::string side;
        if (!(input >> kind >> side)) fail(SuspensionPhase::Parse, value.id,
                                           "missing control kind or side");
        value.kind = parseControlKind(kind);
        value.side = parseSide(side);
        value.minimumCommand = readDouble(input, value.id);
        value.maximumCommand = readDouble(input, value.id);
        value.neutralCommand = readDouble(input, value.id);
        value.maximumRate = readDouble(input, value.id);
        int clamp = 0;
        if (!(input >> clamp) || (clamp != 0 && clamp != 1))
            fail(SuspensionPhase::Parse, value.id,
                 "invalid explicit clamp policy");
        value.clampOutOfRange = clamp == 1;
        const std::size_t targetCount = checkedCount(input, value.id);
        value.provenanceId = readQuoted(input, value.id);
        value.targets.reserve(targetCount);
        for (std::size_t targetIndex = 0; targetIndex < targetCount;
             ++targetIndex) {
            expectToken(input, "TARGET", value.id);
            std::string targetKind;
            if (!(input >> targetKind))
                fail(SuspensionPhase::Parse, value.id,
                     "missing control target kind");
            SuspensionControlTarget target;
            target.kind = parseControlTargetKind(targetKind);
            target.entityId = readQuoted(input, value.id);
            target.travelPerCommand = readDouble(input, value.id);
            target.localDirection = {readDouble(input, value.id),
                                     readDouble(input, value.id),
                                     readDouble(input, value.id)};
            value.targets.push_back(std::move(target));
        }
        definition.controls.push_back(std::move(value));
    }
    expectToken(input, "SOLVER", "solver");
    if (!(input >> definition.solver.lineIterations))
        fail(SuspensionPhase::Parse, "solver", "missing line iterations");
    definition.solver.attachmentTolerance = readDouble(input, "solver");
    definition.solver.minimumLineLength = readDouble(input, "solver");
    definition.solver.maximumLineResidual = readDouble(input, "solver");
    definition.solver.maximumControlWork = readDouble(input, "solver");
    expectToken(input, "GROUND", "ground");
    std::string mode;
    if (!(input >> mode))
        fail(SuspensionPhase::Parse, "ground", "missing ground mode");
    definition.ground.mode = parseGroundMode(mode);
    definition.ground.planeNormal = {readDouble(input, "ground"),
                                     readDouble(input, "ground"),
                                     readDouble(input, "ground")};
    definition.ground.planeOffset = readDouble(input, "ground");
    definition.ground.compliance = readDouble(input, "ground");
    definition.ground.penetrationFraction = readDouble(input, "ground");
    expectToken(input, "END", "end");
    input >> std::ws;
    if (!input.eof())
        fail(SuspensionPhase::Parse, "trailing-data",
             "non-whitespace follows END");
    return validateAndNormalizeSuspensionDefinition(definition);
}

#ifdef SOFTWING_AERODYNAMIC_TEST_ACCESS
std::string AerodynamicVerificationTestAccess::suspensionState(
    const SuspensionSystem& suspension) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(std::numeric_limits<double>::max_digits10);
    const auto vec = [&](const Vec3& value) {
        out << value.x << ' ' << value.y << ' ' << value.z << ' ';
    };
    const auto payload = [&](const RigidPayloadState& value) {
        vec(value.centreOfMassWorld);
        out << value.orientation.w << ' ' << value.orientation.x << ' '
            << value.orientation.y << ' ' << value.orientation.z << ' ';
        vec(value.linearVelocity); vec(value.angularVelocity);
    };
    const auto wrench = [&](const Wrench& value) {
        vec(value.force); vec(value.moment);
    };
    const auto payloadDiagnostics = [&](const PayloadDiagnostics& value) {
        payload(value.state); vec(value.linearMomentum);
        vec(value.angularMomentum);
        out << value.translationalKineticEnergy << ' '
            << value.rotationalKineticEnergy << ' '
            << value.gravitationalEnergy << ' ';
        wrench(value.appliedWrench); wrench(value.lineWrench);
        wrench(value.groundWrench); wrench(value.anchorWrench);
    };
    const auto diagnostic = [&](const SuspensionDiagnostics& value) {
        out << value.registered << ' ' << value.converged << ' '
            << value.allSlack << ' ' << value.anchored << ' '
            << value.grounded << ' ' << value.failedTrial << ' '
            << value.attachmentCount << ' ' << value.junctionCount << ' '
            << value.segmentCount << ' ' << value.tautCount << ' '
            << value.slackCount << ' ' << value.solverIterations << ' '
            << value.maximumResidual << ' '
            << value.maximumGroundPenetration << ' ' << value.elasticEnergy
            << ' ' << value.dampingWork << ' ' << value.controlWork << ' ';
        vec(value.netInternalImpulse); vec(value.netInternalMoment);
        wrench(value.fixedSupportReaction);
        wrench(value.canopySupportReaction); wrench(value.groundReaction);
        out << value.attachmentLoads.size() << ' ';
        for (const auto& [id, load] : value.attachmentLoads) {
            out << std::quoted(id) << ' '; vec(load);
        }
        out << value.groupLoads.size() << ' ';
        for (const auto& [id, load] : value.groupLoads) {
            out << std::quoted(id) << ' '; vec(load);
        }
        out << std::quoted(value.provenance) << ' '
            << static_cast<int>(value.failurePhase) << ' '
            << std::quoted(value.failureEntity) << ' ';
    };
    const auto segment = [&](const RuntimeSuspensionSegment& value) {
        out << std::quoted(value.definition.id) << ' '
            << static_cast<int>(value.definition.from.kind) << ' '
            << std::quoted(value.definition.from.id) << ' '
            << static_cast<int>(value.definition.to.kind) << ' '
            << std::quoted(value.definition.to.id) << ' '
            << value.definition.restLength << ' '
            << value.definition.axialStiffness << ' '
            << value.definition.axialDamping << ' '
            << static_cast<int>(value.definition.side) << ' '
            << value.definition.groups.size() << ' ';
        for (const auto& group : value.definition.groups)
            out << std::quoted(group) << ' ';
        out << std::quoted(value.definition.provenanceId) << ' '
            << static_cast<int>(value.from.definition.kind) << ' '
            << std::quoted(value.from.definition.id) << ' '
            << value.from.nodeIndex.has_value() << ' '
            << value.from.nodeIndex.value_or(0) << ' '
            << value.from.payloadPointIndex.has_value() << ' '
            << value.from.payloadPointIndex.value_or(0) << ' '
            << static_cast<int>(value.to.definition.kind) << ' '
            << std::quoted(value.to.definition.id) << ' '
            << value.to.nodeIndex.has_value() << ' '
            << value.to.nodeIndex.value_or(0) << ' '
            << value.to.payloadPointIndex.has_value() << ' '
            << value.to.payloadPointIndex.value_or(0) << ' '
            << value.attachmentPaths.size() << ' ';
        for (const auto& id : value.attachmentPaths)
            out << std::quoted(id) << ' ';
        out << value.commandedRestLength << ' ' << value.accumulatedLambda
            << ' ';
    };
    const auto control = [&](const SuspensionControlState& value) {
        out << std::quoted(value.id) << ' ' << value.targetCommand << ' '
            << value.actualCommand << ' ' << value.travel << ' '
            << value.signedWork << ' ';
    };
    const auto segmentDiagnostic = [&](const SuspensionSegmentDiagnostics& value) {
        out << std::quoted(value.id) << ' '
            << static_cast<int>(value.from.kind) << ' '
            << std::quoted(value.from.id) << ' '
            << static_cast<int>(value.to.kind) << ' '
            << std::quoted(value.to.id) << ' ' << value.paths.size() << ' ';
        for (const auto& id : value.paths) out << std::quoted(id) << ' ';
        out << value.groups.size() << ' ';
        for (const auto& id : value.groups) out << std::quoted(id) << ' ';
        out << value.length << ' ' << value.commandedRestLength << ' '
            << value.stretch << ' ' << value.strain << ' ' << value.taut
            << ' ' << value.multiplier << ' ' << value.tension << ' '
            << value.residual << ' ' << value.elasticEnergy << ' ';
        vec(value.dampingImpulse);
        out << value.dampingWork << ' ' << value.controlWork << ' ';
        vec(value.fromImpulse); vec(value.toImpulse);
        vec(value.fromMoment); vec(value.toMoment);
    };
    out << "DEFINITION\n" << serializeSuspensionDefinition(
        suspension.definition_);
    out << "IDENTITY " << static_cast<bool>(suspension.lifetimeToken_)
        << " OWNER " << (suspension.owner_ != nullptr) << '\n';
    out << "ATTACHMENTS " << suspension.attachments_.size() << '\n';
    for (const auto& value : suspension.attachments_) {
        out << std::quoted(value.id) << ' ' << std::quoted(value.panelId)
            << ' ' << value.chart.x << ' ' << value.chart.y << ' '
            << value.nodeIndex << ' ';
        vec(value.worldPosition);
        out << std::quoted(value.provenanceId) << '\n';
    }
    out << "JUNCTIONS " << suspension.junctionNodeIndices_.size() << ' ';
    for (const auto node : suspension.junctionNodeIndices_) out << node << ' ';
    out << '\n';
    out << "PAYLOAD "; payload(suspension.payloadState_);
    out << "\nPREVIOUS_PAYLOAD "; payload(suspension.previousPayloadState_);
    out << "\nBASE_HANG " << suspension.baseHangPointPositions_.size() << ' ';
    for (const auto& value : suspension.baseHangPointPositions_) vec(value);
    out << "\nCOMMANDED_HANG "
        << suspension.commandedHangPointPositions_.size() << ' ';
    for (const auto& value : suspension.commandedHangPointPositions_) vec(value);
    out << "\nSEGMENTS " << suspension.segments_.size() << '\n';
    for (const auto& value : suspension.segments_) { segment(value); out << '\n'; }
    out << "CONTROLS " << suspension.controls_.size() << '\n';
    for (const auto& value : suspension.controls_) { control(value); out << '\n'; }
    out << "GROUND " << suspension.groundMultipliers_.size() << ' ';
    for (const auto value : suspension.groundMultipliers_) out << value << ' ';
    out << "\nAPPLIED "; wrench(suspension.appliedWrench_);
    out << "\nGRAVITY "; vec(suspension.currentGravity_);
    out << "\nPENDING " << suspension.pendingSegmentControlWork_.size() << ' ';
    for (const auto value : suspension.pendingSegmentControlWork_)
        out << value << ' ';
    out << "\nSEGMENT_DIAGNOSTICS "
        << suspension.segmentDiagnostics_.size() << '\n';
    for (const auto& value : suspension.segmentDiagnostics_) {
        segmentDiagnostic(value); out << '\n';
    }
    out << "PAYLOAD_DIAGNOSTICS ";
    payloadDiagnostics(suspension.payloadDiagnostics_);
    out << "\nDIAGNOSTICS "; diagnostic(suspension.diagnostics_);
    out << "\nCOMMITTED_DIAGNOSTICS ";
    diagnostic(suspension.committedDiagnostics_);
    out << "\nSNAPSHOT_PAYLOAD "; payload(suspension.snapshotPayloadState_);
    out << "\nSNAPSHOT_PREVIOUS ";
    payload(suspension.snapshotPreviousPayloadState_);
    out << "\nSNAPSHOT_COMMANDED "
        << suspension.snapshotCommandedHangPointPositions_.size() << ' ';
    for (const auto& value : suspension.snapshotCommandedHangPointPositions_)
        vec(value);
    out << "\nSNAPSHOT_SEGMENTS " << suspension.snapshotSegments_.size()
        << '\n';
    for (const auto& value : suspension.snapshotSegments_) {
        segment(value); out << '\n';
    }
    out << "SNAPSHOT_CONTROLS " << suspension.snapshotControls_.size()
        << '\n';
    for (const auto& value : suspension.snapshotControls_) {
        control(value); out << '\n';
    }
    out << "SNAPSHOT_GROUND "
        << suspension.snapshotGroundMultipliers_.size() << ' ';
    for (const auto value : suspension.snapshotGroundMultipliers_)
        out << value << ' ';
    out << "\nSNAPSHOT_GRAVITY "; vec(suspension.snapshotCurrentGravity_);
    out << "\nSNAPSHOT_PENDING "
        << suspension.snapshotPendingSegmentControlWork_.size() << ' ';
    for (const auto value : suspension.snapshotPendingSegmentControlWork_)
        out << value << ' ';
    out << "\nSNAPSHOT_SEGMENT_DIAGNOSTICS "
        << suspension.snapshotSegmentDiagnostics_.size() << '\n';
    for (const auto& value : suspension.snapshotSegmentDiagnostics_) {
        segmentDiagnostic(value); out << '\n';
    }
    out << "SNAPSHOT_PAYLOAD_DIAGNOSTICS ";
    payloadDiagnostics(suspension.snapshotPayloadDiagnostics_);
    out << "\nSNAPSHOT_DIAGNOSTICS ";
    diagnostic(suspension.snapshotDiagnostics_);
    out << "\nHAS_SNAPSHOT " << suspension.hasSnapshot_ << '\n';
    return out.str();
}
#endif

} // namespace softwing
