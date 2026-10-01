#include "helios/math/quaternion.hpp"

#include <gtest/gtest.h>
#include <numbers>

namespace {

using helios::math::conjugate;
using helios::math::from_axis_angle;
using helios::math::from_matrix;
using helios::math::from_rotation_vector;
using helios::math::Matrix3;
using helios::math::norm;
using helios::math::normalized;
using helios::math::Quaternion;
using helios::math::rotate;
using helios::math::to_matrix;
using helios::math::Vector3;

constexpr double k_quarter_turn = std::numbers::pi / 2.0;

TEST(Quaternion, RotatesVectorsRightHandedly) {
    const Quaternion about_z = from_axis_angle({0.0, 0.0, 1.0}, k_quarter_turn);
    const Vector3 turned = rotate(about_z, {1.0, 0.0, 0.0});
    EXPECT_NEAR(turned.x, 0.0, 1e-15);
    EXPECT_NEAR(turned.y, 1.0, 1e-15);
    EXPECT_NEAR(turned.z, 0.0, 1e-15);
    EXPECT_EQ(rotate(Quaternion{}, Vector3{1.0, 2.0, 3.0}), (Vector3{1.0, 2.0, 3.0}));
}

TEST(Quaternion, ProductAppliesTheRightHandFactorFirst) {
    const Quaternion about_x = from_axis_angle({1.0, 0.0, 0.0}, k_quarter_turn);
    const Quaternion about_z = from_axis_angle({0.0, 0.0, 1.0}, k_quarter_turn);
    // x → (about z) → y → (about x) → z.
    const Vector3 turned = rotate(about_x * about_z, {1.0, 0.0, 0.0});
    EXPECT_NEAR(norm(turned - Vector3{0.0, 0.0, 1.0}), 0.0, 1e-14);
    const Vector3 back = rotate(conjugate(about_x * about_z), turned);
    EXPECT_NEAR(norm(back - Vector3{1.0, 0.0, 0.0}), 0.0, 1e-14);
}

TEST(Quaternion, AgreesWithItsMatrixAndComesBackFromIt) {
    for (const Vector3& rotation_rad :
         {Vector3{0.3, -1.1, 0.7}, Vector3{3.0, 0.2, 0.1}, Vector3{0.0, 3.1, 0.0}, Vector3{0.1, 0.1, -3.0},
          Vector3{0.0, 0.0, 0.0}}) {
        const Quaternion rotation = from_rotation_vector(rotation_rad);
        const Matrix3 matrix = to_matrix(rotation);
        const Vector3 vector{0.4, -2.0, 1.3};
        EXPECT_NEAR(norm(matrix * vector - rotate(rotation, vector)), 0.0, 1e-14);
        // A rotation and its negative are the same rotation.
        const Quaternion recovered = from_matrix(matrix);
        EXPECT_NEAR(norm(rotate(recovered, vector) - rotate(rotation, vector)), 0.0, 1e-14);
    }
}

TEST(Quaternion, NormalisingAZeroQuaternionGivesTheIdentity) {
    EXPECT_EQ(normalized(Quaternion{0.0, 0.0, 0.0, 0.0}), Quaternion{});
    const Quaternion scaled = normalized(Quaternion{2.0, 0.0, 0.0, 0.0});
    EXPECT_EQ(scaled, Quaternion{});
}

} // namespace
