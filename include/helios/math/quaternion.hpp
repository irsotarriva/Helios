#ifndef HELIOS_MATH_QUATERNION_HPP
#define HELIOS_MATH_QUATERNION_HPP

#include "helios/math/matrix3.hpp"
#include "helios/math/vector3.hpp"

#include <cmath>

namespace helios::math {

// A rotation as a unit quaternion w + xi + yj + zk (Hamilton convention). It is an *active*
// rotation: rotate(q, v) turns the vector v, and for an orientation it takes coordinates in the
// object's own axes to the axes the orientation is given in.
struct Quaternion {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    [[nodiscard]] friend constexpr bool operator==(const Quaternion&, const Quaternion&) noexcept = default;
};

// The rotation `rhs` followed by `lhs`.
[[nodiscard]] constexpr Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs) noexcept {
    return {lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
            lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
            lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w};
}

// The inverse rotation, for a unit quaternion.
[[nodiscard]] constexpr Quaternion conjugate(const Quaternion& rotation) noexcept {
    return {rotation.w, -rotation.x, -rotation.y, -rotation.z};
}

// Unit length; the identity for a zero quaternion.
[[nodiscard]] inline Quaternion normalized(const Quaternion& rotation) noexcept {
    const double length = std::sqrt(rotation.w * rotation.w + rotation.x * rotation.x
                                    + rotation.y * rotation.y + rotation.z * rotation.z);
    if (!(length > 0.0)) {
        return {};
    }
    return {rotation.w / length, rotation.x / length, rotation.y / length, rotation.z / length};
}

[[nodiscard]] constexpr Vector3 rotate(const Quaternion& rotation, const Vector3& vector) noexcept {
    // v' = v + 2 u × (u × v + w v), with u the vector part.
    const Vector3 axis{rotation.x, rotation.y, rotation.z};
    return vector + 2.0 * cross(axis, cross(axis, vector) + rotation.w * vector);
}

// By `angle_rad` about `unit_axis` (right-handed).
[[nodiscard]] inline Quaternion from_axis_angle(const Vector3& unit_axis, double angle_rad) noexcept {
    const double sine = std::sin(0.5 * angle_rad);
    return {std::cos(0.5 * angle_rad), sine * unit_axis.x, sine * unit_axis.y, sine * unit_axis.z};
}

// From a rotation vector: its direction is the axis, its length the angle. Exact for any length.
[[nodiscard]] inline Quaternion from_rotation_vector(const Vector3& rotation_rad) noexcept {
    const double angle_rad = norm(rotation_rad);
    if (!(angle_rad > 0.0)) {
        return {};
    }
    return from_axis_angle(rotation_rad / angle_rad, angle_rad);
}

// The same rotation as a matrix: to_matrix(q) * v == rotate(q, v).
[[nodiscard]] constexpr Matrix3 to_matrix(const Quaternion& rotation) noexcept {
    const double w = rotation.w;
    const double x = rotation.x;
    const double y = rotation.y;
    const double z = rotation.z;
    return Matrix3{{1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y),
                    2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
                    2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)}};
}

// From a rotation matrix (orthonormal, determinant +1), by Shepperd's method: the largest of
// the four candidates is used as the divisor, so no rotation is ill-conditioned.
[[nodiscard]] inline Quaternion from_matrix(const Matrix3& matrix) noexcept {
    const double trace = matrix(0, 0) + matrix(1, 1) + matrix(2, 2);
    Quaternion rotation;
    if (trace > 0.0) {
        const double scale = 2.0 * std::sqrt(trace + 1.0);
        rotation = {0.25 * scale, (matrix(2, 1) - matrix(1, 2)) / scale,
                    (matrix(0, 2) - matrix(2, 0)) / scale, (matrix(1, 0) - matrix(0, 1)) / scale};
    } else if (matrix(0, 0) > matrix(1, 1) && matrix(0, 0) > matrix(2, 2)) {
        const double scale = 2.0 * std::sqrt(1.0 + matrix(0, 0) - matrix(1, 1) - matrix(2, 2));
        rotation = {(matrix(2, 1) - matrix(1, 2)) / scale, 0.25 * scale,
                    (matrix(0, 1) + matrix(1, 0)) / scale, (matrix(0, 2) + matrix(2, 0)) / scale};
    } else if (matrix(1, 1) > matrix(2, 2)) {
        const double scale = 2.0 * std::sqrt(1.0 + matrix(1, 1) - matrix(0, 0) - matrix(2, 2));
        rotation = {(matrix(0, 2) - matrix(2, 0)) / scale, (matrix(0, 1) + matrix(1, 0)) / scale,
                    0.25 * scale, (matrix(1, 2) + matrix(2, 1)) / scale};
    } else {
        const double scale = 2.0 * std::sqrt(1.0 + matrix(2, 2) - matrix(0, 0) - matrix(1, 1));
        rotation = {(matrix(1, 0) - matrix(0, 1)) / scale, (matrix(0, 2) + matrix(2, 0)) / scale,
                    (matrix(1, 2) + matrix(2, 1)) / scale, 0.25 * scale};
    }
    return normalized(rotation);
}

} // namespace helios::math

#endif // HELIOS_MATH_QUATERNION_HPP
