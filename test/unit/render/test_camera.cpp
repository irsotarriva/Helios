#include "helios/render/camera.hpp"
#include "helios/render/mesh.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>

namespace {

using helios::math::dot;
using helios::math::norm;
using helios::math::Vector3;
using helios::render::Float3;
using helios::render::make_icosphere;
using helios::render::multiply;
using helios::render::OrbitCamera;
using helios::render::project_to_screen;
using helios::render::reversed_infinite_projection;
using helios::render::to_camera_space;
using helios::render::view_matrix;

TEST(OrbitCamera, LooksAtTheFocusWithAnOrthonormalBasis) {
    OrbitCamera camera;
    camera.yaw_rad = 0.7;
    camera.pitch_rad = -0.4;
    camera.distance_m = 1e11;
    const auto rotation = camera.universe_to_camera();
    const Vector3 right{rotation(0, 0), rotation(0, 1), rotation(0, 2)};
    const Vector3 up{rotation(1, 0), rotation(1, 1), rotation(1, 2)};
    const Vector3 backward{rotation(2, 0), rotation(2, 1), rotation(2, 2)};
    EXPECT_NEAR(norm(right), 1.0, 1e-15);
    EXPECT_NEAR(dot(right, up), 0.0, 1e-15);
    EXPECT_NEAR(dot(up, backward), 0.0, 1e-15);
    EXPECT_NEAR(right.z, 0.0, 1e-15) << "no roll: the horizon stays level";
    // The focus (at −offset from the camera) is straight ahead, along −z.
    const Vector3 focus_in_camera = rotation * (-camera.offset_from_focus_m());
    EXPECT_NEAR(focus_in_camera.x / camera.distance_m, 0.0, 1e-15);
    EXPECT_NEAR(focus_in_camera.z / camera.distance_m, -1.0, 1e-15);
}

TEST(OrbitCamera, PitchAndZoomAreClamped) {
    OrbitCamera camera;
    camera.orbit(0.0, 10.0);
    EXPECT_EQ(camera.pitch_rad, OrbitCamera::k_max_pitch_rad);
    camera.zoom(1e-30, 6.4e6);
    EXPECT_EQ(camera.distance_m, 6.4e6);
    camera.zoom(1e30, 0.0);
    EXPECT_EQ(camera.distance_m, OrbitCamera::k_max_distance_m);
}

TEST(FloatingOrigin, CameraRelativeFloatsKeepMetrePrecisionFarFromTheOrigin) {
    // A point 1 AU from the focus, seen from 10 m away: the float result must be good to
    // millimetres, which rounding the absolute 1.5e11 m position to float (~16 km) never is.
    const Vector3 point_m{1.495978707e11, 1.0e3, -2.0e3};
    const Vector3 camera_m{1.495978707e11 - 10.0, 1.0e3, -2.0e3};
    const Float3 relative = to_camera_space(point_m, camera_m);
    EXPECT_NEAR(relative.x, 10.0F, 1e-3F);
    EXPECT_NEAR(relative.y, 0.0F, 1e-6F);
    const auto naive = static_cast<float>(point_m.x) - static_cast<float>(camera_m.x);
    EXPECT_GT(std::abs(naive - 10.0F), 1.0F) << "the test must be one float alone would fail";
}

TEST(ReversedInfiniteProjection, NearPlaneMapsToOneAndInfinityToZero) {
    for (const bool homogeneous : {false, true}) {
        const auto projection = reversed_infinite_projection(1.0, 16.0 / 9.0, 0.5, homogeneous);
        const auto near_point = project_to_screen(Float3{0.0F, 0.0F, -0.5F}, projection, 100.0F, 100.0F);
        const auto far_point = project_to_screen(Float3{0.0F, 0.0F, -1e20F}, projection, 100.0F, 100.0F);
        ASSERT_TRUE(near_point.has_value() && far_point.has_value());
        EXPECT_NEAR(near_point->depth, 1.0F, 1e-6F);
        EXPECT_NEAR(far_point->depth, homogeneous ? -1.0F : 0.0F, 1e-6F);
        EXPECT_NEAR(near_point->x, 50.0F, 1e-4F);
        EXPECT_FALSE(project_to_screen(Float3{0.0F, 0.0F, 5.0F}, projection, 100.0F, 100.0F).has_value());
    }
}

TEST(ReversedInfiniteProjection, ScreenAxesFollowTheCamera) {
    OrbitCamera camera;
    camera.yaw_rad = 0.0;
    camera.pitch_rad = 0.0;
    camera.distance_m = 10.0;
    const auto view_projection = multiply(reversed_infinite_projection(1.0, 1.0, 0.1, false),
                                          view_matrix(camera.universe_to_camera()));
    // Camera on +x looking at the origin with +z up: +y (universe) is to the right on screen,
    // +z is up.
    const Vector3 camera_m = camera.offset_from_focus_m();
    const auto to_right =
        project_to_screen(to_camera_space({0.0, 1.0, 0.0}, camera_m), view_projection, 200, 200);
    const auto upwards =
        project_to_screen(to_camera_space({0.0, 0.0, 1.0}, camera_m), view_projection, 200, 200);
    ASSERT_TRUE(to_right.has_value() && upwards.has_value());
    EXPECT_GT(to_right->x, 100.0F);
    EXPECT_LT(upwards->y, 100.0F);
}

TEST(ReversedInfiniteProjection, AFieldOfViewOffTheAxisPutsItsEdgesOnTheEdgesOfThePicture) {
    // Symmetric, it is the projection by angle and aspect ratio.
    const double half_height = std::atan(0.5);
    const double half_width = std::atan(1.0);
    const helios::render::FieldOfView even{
        .left_rad = -half_width, .right_rad = half_width, .up_rad = half_height, .down_rad = -half_height};
    for (const bool homogeneous : {false, true}) {
        const auto by_angle = reversed_infinite_projection(2.0 * half_height, 2.0, 0.5, homogeneous);
        const auto by_edges = reversed_infinite_projection(even, 0.5, homogeneous);
        for (std::size_t element = 0; element < by_angle.size(); ++element) {
            EXPECT_NEAR(by_edges.at(element), by_angle.at(element), 1e-6F) << element;
        }
    }

    // An eye of a headset sees further outwards than towards the nose.
    const helios::render::FieldOfView left_eye{
        .left_rad = -0.9, .right_rad = 0.7, .up_rad = 0.8, .down_rad = -0.85};
    const auto projection = reversed_infinite_projection(left_eye, 0.1, false);
    const auto at = [&](double x, double y) {
        return project_to_screen(Float3{static_cast<float>(x), static_cast<float>(y), -1.0F}, projection,
                                 200.0F, 100.0F);
    };
    const auto left_edge = at(std::tan(-0.9), 0.0);
    const auto right_edge = at(std::tan(0.7), 0.0);
    const auto top_edge = at(0.0, std::tan(0.8));
    const auto bottom_edge = at(0.0, std::tan(-0.85));
    const auto axis = at(0.0, 0.0);
    ASSERT_TRUE(left_edge.has_value() && right_edge.has_value() && top_edge.has_value()
                && bottom_edge.has_value() && axis.has_value());
    EXPECT_NEAR(left_edge->x, 0.0F, 1e-3F);
    EXPECT_NEAR(right_edge->x, 200.0F, 1e-3F);
    EXPECT_NEAR(top_edge->y, 0.0F, 1e-3F);
    EXPECT_NEAR(bottom_edge->y, 100.0F, 1e-3F);
    EXPECT_GT(axis->x, 100.0F) << "the axis is towards the nose side of the picture";
    const auto near_point = project_to_screen(Float3{0.0F, 0.0F, -0.1F}, projection, 200.0F, 100.0F);
    ASSERT_TRUE(near_point.has_value());
    EXPECT_NEAR(near_point->depth, 1.0F, 1e-6F);
}

TEST(ViewMatrix, ADisplacedCameraSeesThePointItStandsOnAtItsOrigin) {
    OrbitCamera camera;
    camera.yaw_rad = 0.4;
    camera.pitch_rad = -0.2;
    const auto rotation = camera.universe_to_camera();
    const Vector3 offset_m{0.032, -0.01, 0.2};
    const auto view = view_matrix(rotation, offset_m);
    // Geometry is relative to the head; the eye is at `offset_m` from it.
    const Vector3 point_m{3.0, -2.0, 0.5};
    const Vector3 seen = rotation * (point_m - offset_m);
    const std::array<double, 3> expected{seen.x, seen.y, seen.z};
    const std::array<float, 4> column{static_cast<float>(point_m.x), static_cast<float>(point_m.y),
                                      static_cast<float>(point_m.z), 1.0F};
    for (std::size_t row = 0; row < 3; ++row) {
        float sum = 0.0F;
        for (std::size_t inner = 0; inner < 4; ++inner) {
            sum += view.at(inner * 4 + row) * column.at(inner);
        }
        EXPECT_NEAR(sum, expected.at(row), 1e-5);
    }
    EXPECT_EQ(view_matrix(rotation, Vector3{}), view_matrix(rotation));
}

TEST(Icosphere, IsAClosedUnitSphere) {
    const auto mesh = make_icosphere(3);
    EXPECT_EQ(mesh.indices.size(), 20U * 64U * 3U);
    EXPECT_EQ(mesh.vertices.size(), 642U); // 10·4^n + 2
    for (const Float3& vertex : mesh.vertices) {
        EXPECT_NEAR(std::sqrt(vertex.x * vertex.x + vertex.y * vertex.y + vertex.z * vertex.z), 1.0F, 1e-6F);
    }
    // Outward winding: each triangle's normal points away from the centre.
    for (std::size_t triangle = 0; triangle < mesh.indices.size(); triangle += 3) {
        const Float3& a = mesh.vertices[mesh.indices[triangle]];
        const Float3& b = mesh.vertices[mesh.indices[triangle + 1]];
        const Float3& c = mesh.vertices[mesh.indices[triangle + 2]];
        const Vector3 ab{b.x - a.x, b.y - a.y, b.z - a.z};
        const Vector3 ac{c.x - a.x, c.y - a.y, c.z - a.z};
        EXPECT_GT(dot(helios::math::cross(ab, ac), Vector3{a.x, a.y, a.z}), 0.0) << triangle;
    }
}

} // namespace
