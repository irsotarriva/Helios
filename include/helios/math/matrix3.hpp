#ifndef HELIOS_MATH_MATRIX3_HPP
#define HELIOS_MATH_MATRIX3_HPP

#include "helios/math/vector3.hpp"

#include <array>
#include <cmath>
#include <cstddef>

namespace helios::math {

// Row-major 3×3 double matrix, used for frame rotations.
struct Matrix3 {
    std::array<double, 9> elements{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    [[nodiscard]] constexpr double operator()(std::size_t row, std::size_t column) const noexcept {
        return elements[(row * 3) + column];
    }
    [[nodiscard]] constexpr double& operator()(std::size_t row, std::size_t column) noexcept {
        return elements[(row * 3) + column];
    }
};

[[nodiscard]] constexpr Vector3 operator*(const Matrix3& matrix, const Vector3& vector) noexcept {
    return {matrix(0, 0) * vector.x + matrix(0, 1) * vector.y + matrix(0, 2) * vector.z,
            matrix(1, 0) * vector.x + matrix(1, 1) * vector.y + matrix(1, 2) * vector.z,
            matrix(2, 0) * vector.x + matrix(2, 1) * vector.y + matrix(2, 2) * vector.z};
}

[[nodiscard]] constexpr Matrix3 operator*(const Matrix3& lhs, const Matrix3& rhs) noexcept {
    Matrix3 product;
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            product(row, column) =
                lhs(row, 0) * rhs(0, column) + lhs(row, 1) * rhs(1, column) + lhs(row, 2) * rhs(2, column);
        }
    }
    return product;
}

[[nodiscard]] constexpr Matrix3 transpose(const Matrix3& matrix) noexcept {
    Matrix3 transposed;
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            // NOLINTNEXTLINE(readability-suspicious-call-argument): swapping indices is the point.
            transposed(row, column) = matrix(column, row);
        }
    }
    return transposed;
}

// Passive (frame) rotations, as used in astronomy: the coordinates of a fixed vector in a
// frame rotated by `angle_rad` about the axis. R1 = about x, R3 = about z.
[[nodiscard]] inline Matrix3 frame_rotation_x(double angle_rad) noexcept {
    const double cosine = std::cos(angle_rad);
    const double sine = std::sin(angle_rad);
    return Matrix3{{1.0, 0.0, 0.0, 0.0, cosine, sine, 0.0, -sine, cosine}};
}

[[nodiscard]] inline Matrix3 frame_rotation_z(double angle_rad) noexcept {
    const double cosine = std::cos(angle_rad);
    const double sine = std::sin(angle_rad);
    return Matrix3{{cosine, sine, 0.0, -sine, cosine, 0.0, 0.0, 0.0, 1.0}};
}

} // namespace helios::math

#endif // HELIOS_MATH_MATRIX3_HPP
