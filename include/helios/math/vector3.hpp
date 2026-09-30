#ifndef HELIOS_MATH_VECTOR3_HPP
#define HELIOS_MATH_VECTOR3_HPP

#include <cmath>

namespace helios::math {

// A double-precision 3-vector. It carries no unit: the *name* of each variable does
// (CODING_STANDARDS §5.1), e.g. `position_m`, `velocity_m_s`.
struct Vector3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    [[nodiscard]] friend constexpr bool operator==(const Vector3&, const Vector3&) noexcept = default;

    constexpr Vector3& operator+=(const Vector3& other) noexcept {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
    constexpr Vector3& operator-=(const Vector3& other) noexcept {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }
    constexpr Vector3& operator*=(double scale) noexcept {
        x *= scale;
        y *= scale;
        z *= scale;
        return *this;
    }
};

[[nodiscard]] constexpr Vector3 operator+(Vector3 lhs, const Vector3& rhs) noexcept {
    return lhs += rhs;
}
[[nodiscard]] constexpr Vector3 operator-(Vector3 lhs, const Vector3& rhs) noexcept {
    return lhs -= rhs;
}
[[nodiscard]] constexpr Vector3 operator-(const Vector3& value) noexcept {
    return {-value.x, -value.y, -value.z};
}
[[nodiscard]] constexpr Vector3 operator*(Vector3 vector, double scale) noexcept {
    return vector *= scale;
}
[[nodiscard]] constexpr Vector3 operator*(double scale, Vector3 vector) noexcept {
    return vector *= scale;
}
[[nodiscard]] constexpr Vector3 operator/(const Vector3& vector, double divisor) noexcept {
    return {vector.x / divisor, vector.y / divisor, vector.z / divisor};
}

[[nodiscard]] constexpr double dot(const Vector3& lhs, const Vector3& rhs) noexcept {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

[[nodiscard]] constexpr Vector3 cross(const Vector3& lhs, const Vector3& rhs) noexcept {
    return {lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x};
}

[[nodiscard]] constexpr double squared_norm(const Vector3& vector) noexcept {
    return dot(vector, vector);
}

// Rationale: std::hypot(x, y, z) avoids overflow/underflow for extreme magnitudes
// (galactic-scale coordinates squared overflow nothing in double, but tiny ones can underflow).
[[nodiscard]] inline double norm(const Vector3& vector) noexcept {
    return std::hypot(vector.x, vector.y, vector.z);
}

} // namespace helios::math

#endif // HELIOS_MATH_VECTOR3_HPP
