#pragma once

#include <cmath>

namespace softwing {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    constexpr Vec3() = default;
    constexpr Vec3(double xValue, double yValue, double zValue)
        : x(xValue), y(yValue), z(zValue) {}

    constexpr Vec3 operator+() const { return *this; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }

    constexpr Vec3& operator+=(const Vec3& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    constexpr Vec3& operator-=(const Vec3& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    constexpr Vec3& operator*=(double scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    constexpr Vec3& operator/=(double scalar) {
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }
};

constexpr Vec3 operator+(Vec3 lhs, const Vec3& rhs) { return lhs += rhs; }
constexpr Vec3 operator-(Vec3 lhs, const Vec3& rhs) { return lhs -= rhs; }
constexpr Vec3 operator*(Vec3 value, double scalar) { return value *= scalar; }
constexpr Vec3 operator*(double scalar, Vec3 value) { return value *= scalar; }
constexpr Vec3 operator/(Vec3 value, double scalar) { return value /= scalar; }

constexpr double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

constexpr double lengthSquared(const Vec3& value) { return dot(value, value); }

inline double length(const Vec3& value) { return std::sqrt(lengthSquared(value)); }

inline Vec3 normalized(const Vec3& value) {
    const double magnitude = length(value);
    return magnitude > 0.0 ? value / magnitude : Vec3{};
}

} // namespace softwing

