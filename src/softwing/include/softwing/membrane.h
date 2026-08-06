#pragma once

#include "softwing/vec3.h"

#include <array>
#include <cstddef>

namespace softwing {

class SoftBody;

struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    constexpr Vec2() = default;
    constexpr Vec2(double xValue, double yValue) : x(xValue), y(yValue) {}

    constexpr Vec2& operator+=(const Vec2& rhs) {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }
    constexpr Vec2& operator-=(const Vec2& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }
    constexpr Vec2& operator*=(double scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }
};

constexpr Vec2 operator+(Vec2 lhs, const Vec2& rhs) { return lhs += rhs; }
constexpr Vec2 operator-(Vec2 lhs, const Vec2& rhs) { return lhs -= rhs; }
constexpr Vec2 operator*(Vec2 value, double scalar) { return value *= scalar; }
constexpr Vec2 operator*(double scalar, Vec2 value) { return value *= scalar; }
constexpr double dot(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;
}
constexpr double cross(const Vec2& a, const Vec2& b) {
    return a.x * b.y - a.y * b.x;
}
constexpr double lengthSquared(const Vec2& value) { return dot(value, value); }

struct Matrix2 {
    double m00 = 0.0;
    double m01 = 0.0;
    double m10 = 0.0;
    double m11 = 0.0;

    [[nodiscard]] constexpr double determinant() const {
        return m00 * m11 - m01 * m10;
    }
    [[nodiscard]] constexpr Vec2 operator*(const Vec2& value) const {
        return {m00 * value.x + m01 * value.y,
                m10 * value.x + m11 * value.y};
    }
};

[[nodiscard]] Matrix2 checkedInverse(const Matrix2& matrix);

struct SymmetricMatrix3 {
    double xx = 0.0;
    double yy = 0.0;
    double zz = 0.0;
    double xy = 0.0;
    double xz = 0.0;
    double yz = 0.0;

    [[nodiscard]] double determinant() const;
    [[nodiscard]] Vec3 operator*(const Vec3& value) const;
};

[[nodiscard]] SymmetricMatrix3 operator+(const SymmetricMatrix3& lhs,
                                         const SymmetricMatrix3& rhs);
[[nodiscard]] SymmetricMatrix3 operator*(double scalar,
                                         const SymmetricMatrix3& matrix);
[[nodiscard]] bool isPositiveDefinite(const SymmetricMatrix3& matrix);
[[nodiscard]] SymmetricMatrix3 checkedInverse(const SymmetricMatrix3& matrix);
[[nodiscard]] Vec3 checkedSolve(const SymmetricMatrix3& matrix,
                                const Vec3& rightHandSide);

enum class MaterialRole {
    Bulk,
    Seam,
    Reinforcement,
};

struct OrthotropicMembraneMaterial {
    double warpStiffness = 0.0;
    double weftStiffness = 0.0;
    double couplingStiffness = 0.0;
    double shearStiffness = 0.0;
    double warpPreTension = 0.0;
    double weftPreTension = 0.0;
    double dampingTime = 0.0;
    // Fraction of tensile tangent stiffness retained by a normal material
    // direction while its Green strain is compressive. 1.0 is the exact
    // historical bilateral membrane. Lower values approximate wrinkling.
    double compressionStiffnessRatio = 1.0;

    [[nodiscard]] SymmetricMatrix3 stiffnessMatrix() const;
    [[nodiscard]] SymmetricMatrix3 complianceMatrix() const;
};

// State-dependent constitutive matrix used by diagnostics and XPBD. The
// ratio==1 path returns stiffnessMatrix() directly and preserves the original
// arithmetic. Otherwise an SPD-safe D*K*D scaling is used.
[[nodiscard]] SymmetricMatrix3 effectiveMembraneStiffness(
    const OrthotropicMembraneMaterial& material,
    const Vec3& greenStrain);

void validateOrthotropicMembraneMaterial(
    const OrthotropicMembraneMaterial& material);

struct MembraneElementDefinition {
    std::size_t triangle = 0;
    std::array<Vec2, 3> chart{};
    OrthotropicMembraneMaterial material;
    MaterialRole role = MaterialRole::Bulk;
};

struct MembraneElement {
    std::size_t triangle = 0;
    std::array<Vec2, 3> chart{};
    OrthotropicMembraneMaterial material;
    MaterialRole role = MaterialRole::Bulk;
    double referenceArea = 0.0;
    Matrix2 inverseReferenceMatrix;
    Vec3 multiplier;
    Vec3 solverResultantEstimate;
    double normalizedResidual = 0.0;
    // Derived from `material`, which is write-once at construction. Cached
    // because the XPBD sweep needs it once per element per iteration and
    // recomputing it there costs a validation pass plus a 3x3 inverse.
    // Serialization deliberately omits it: it is rebuilt from `material`.
    SymmetricMatrix3 complianceMatrix;
};

struct MembraneElementDiagnostics {
    Vec3 deformationWarp;
    Vec3 deformationWeft;
    Vec3 greenStrain;
    Vec3 constitutiveResultant;
    double elasticEnergy = 0.0;
    MaterialRole role = MaterialRole::Bulk;
    std::array<Vec3, 3> nodalForces{};
    Vec3 totalInternalForce;
    Vec3 totalInternalMoment;
    Vec3 solverResultantEstimate;
    double normalizedResidual = 0.0;
};

struct MembraneGroupDiagnostics {
    double elasticEnergy = 0.0;
    std::array<double, 3> energyByRole{};
    Vec3 totalInternalForce;
    Vec3 totalInternalMoment;
    double maximumAbsoluteStrain = 0.0;
    double maximumResidual = 0.0;
};

class MembraneGroup {
public:
    [[nodiscard]] std::size_t firstElement() const { return firstElement_; }
    [[nodiscard]] std::size_t elementCount() const { return elementCount_; }

private:
    friend class SoftBody;

    MembraneGroup(const SoftBody* owner,
                  std::size_t firstElement,
                  std::size_t elementCount)
        : owner_(owner),
          firstElement_(firstElement),
          elementCount_(elementCount) {}

    const SoftBody* owner_ = nullptr;
    std::size_t firstElement_ = 0;
    std::size_t elementCount_ = 0;
};

} // namespace softwing
