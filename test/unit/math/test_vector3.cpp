#include "helios/math/vector3.hpp"

#include <gtest/gtest.h>

namespace {

using helios::math::cross;
using helios::math::dot;
using helios::math::norm;
using helios::math::squared_norm;
using helios::math::Vector3;

TEST(Vector3, ArithmeticIsComponentWise) {
    constexpr Vector3 k_lhs{1.0, 2.0, 3.0};
    constexpr Vector3 k_rhs{4.0, -5.0, 6.0};
    static_assert(k_lhs + k_rhs == Vector3{5.0, -3.0, 9.0});
    static_assert(k_lhs - k_rhs == Vector3{-3.0, 7.0, -3.0});
    static_assert(2.0 * k_lhs == Vector3{2.0, 4.0, 6.0});
    static_assert(-k_lhs == Vector3{-1.0, -2.0, -3.0});
    static_assert(dot(k_lhs, k_rhs) == 12.0);
    static_assert(squared_norm(k_lhs) == 14.0);
}

TEST(Vector3, CrossProductIsRightHanded) {
    static_assert(cross(Vector3{1.0, 0.0, 0.0}, Vector3{0.0, 1.0, 0.0}) == Vector3{0.0, 0.0, 1.0});
    static_assert(cross(Vector3{0.0, 1.0, 0.0}, Vector3{1.0, 0.0, 0.0}) == Vector3{0.0, 0.0, -1.0});
}

TEST(Vector3, NormHandlesExtremeMagnitudes) {
    EXPECT_DOUBLE_EQ(norm(Vector3{3.0, 4.0, 12.0}), 13.0);
    EXPECT_DOUBLE_EQ(norm(Vector3{3e200, 4e200, 0.0}), 5e200);
    EXPECT_DOUBLE_EQ(norm(Vector3{3e-200, 4e-200, 0.0}), 5e-200);
}

} // namespace
