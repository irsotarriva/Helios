#include "helios/render/scene_geometry.hpp"

#include <gtest/gtest.h>

namespace {

using helios::math::Vector3;
using helios::render::body_colour;
using helios::render::build_frame_geometry;
using helios::render::rgba;
using helios::sim::BodyView;
using helios::sim::LineKind;
using helios::sim::LineView;
using helios::sim::SceneSnapshot;

[[nodiscard]] SceneSnapshot two_body_snapshot() {
    SceneSnapshot snapshot;
    snapshot.bodies.push_back(
        BodyView{.id = {0}, .name = "Sun", .position_m = {-1.5e11, 0.0, 0.0}, .radius_m = 7e8});
    snapshot.bodies.push_back(BodyView{.id = {1},
                                       .name = "Earth",
                                       .position_m = {},
                                       .radius_m = 6.4e6,
                                       .domain_parent = helios::bodies::BodyId{0}});
    LineView orbit{
        .kind = LineKind::BodyOrbit, .owner = 1, .frame_body = {0}, .points_m = {}, .closed = true};
    for (int index = 0; index < 8; ++index) {
        const double angle = index * 0.785398163;
        orbit.points_m.push_back(Vector3{-1.5e11 + 1.5e11 * std::cos(angle), 1.5e11 * std::sin(angle), 0.0});
    }
    snapshot.lines.push_back(orbit);
    return snapshot;
}

TEST(SceneGeometry, BodiesAreLitByTheStarAndPlacedRelativeToTheCamera) {
    const SceneSnapshot snapshot = two_body_snapshot();
    const Vector3 camera_m{0.0, 0.0, 2e7};
    const auto geometry = build_frame_geometry(snapshot, camera_m);
    ASSERT_EQ(geometry.bodies.size(), 2U);
    EXPECT_TRUE(geometry.bodies[0].emissive);
    EXPECT_FALSE(geometry.bodies[1].emissive);
    EXPECT_FLOAT_EQ(geometry.bodies[1].centre.z, -2e7F);
    EXPECT_NEAR(geometry.bodies[1].sun_direction.x, -1.0F, 1e-6F);
    EXPECT_FLOAT_EQ(geometry.bodies[1].model[0], 6.4e6F); // identity rotation × radius
    EXPECT_EQ(geometry.lines.size(), 16U);                // closed octagon: 8 segments
    EXPECT_EQ(geometry.labels.size(), 2U);
}

TEST(SceneGeometry, TinyOrbitsAreCulledWhenFarAway) {
    const SceneSnapshot snapshot = two_body_snapshot();
    const auto geometry = build_frame_geometry(snapshot, Vector3{0.0, 0.0, 1e15});
    EXPECT_TRUE(geometry.lines.empty());
}

TEST(SceneGeometry, StockColoursAndStableFallbacks) {
    EXPECT_EQ(body_colour("Earth"), rgba(70, 130, 220));
    EXPECT_EQ(body_colour("Kerbin-like"), body_colour("Kerbin-like"));
    EXPECT_NE(body_colour("Alpha"), body_colour("Beta"));
}

} // namespace
