#include "helios/physics/world.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace {

using helios::core::ErrorCode;
using helios::math::Matrix3;
using helios::math::norm;
using helios::math::rotate;
using helios::math::Vector3;
using helios::physics::BodyDescription;
using helios::physics::BodyId;
using helios::physics::BodyState;
using helios::physics::ShapePiece;
using helios::physics::World;

constexpr double k_step_s = 1.0 / 128.0;

[[nodiscard]] Matrix3 diagonal(double x, double y, double z) {
    return Matrix3{{x, 0.0, 0.0, 0.0, y, 0.0, 0.0, 0.0, z}};
}

// A 100 kg cylinder 2 m long and 0.5 m in radius, lying along its x axis.
[[nodiscard]] BodyDescription cylinder() {
    return BodyDescription{.pieces = {ShapePiece{.radius_m = 0.5, .half_length_m = 1.0}},
                           .mass_kg = 100.0,
                           .inertia_kg_m2 = diagonal(12.5, 39.6, 39.6),
                           .state = {}};
}

void run(World& world, int steps) {
    for (int step = 0; step < steps; ++step) {
        ASSERT_TRUE(world.step(k_step_s).has_value());
    }
}

TEST(PhysicsWorld, AFreeBodyKeepsItsVelocityAndSpin) {
    World world = World::make().value();
    BodyDescription description = cylinder();
    description.state.velocity_m_s = {3.0, 0.0, -1.0};
    description.state.angular_velocity_rad_s = {0.5, 0.0, 0.0}; // about its axis of symmetry
    const BodyId body = world.add_body(description).value();
    run(world, 256);
    const BodyState state = world.state(body).value();
    EXPECT_NEAR(state.position_m.x, 6.0, 1e-4);
    EXPECT_NEAR(state.position_m.z, -2.0, 1e-4);
    EXPECT_NEAR(norm(state.velocity_m_s - Vector3{3.0, 0.0, -1.0}), 0.0, 1e-5);
    EXPECT_NEAR(state.angular_velocity_rad_s.x, 0.5, 1e-5);
    // One radian about x in two seconds.
    const Vector3 turned = rotate(state.orientation, {0.0, 1.0, 0.0});
    EXPECT_NEAR(turned.y, std::cos(1.0), 1e-3);
    EXPECT_NEAR(turned.z, std::sin(1.0), 1e-3);
    EXPECT_FALSE(world.touching(body));
}

TEST(PhysicsWorld, ForcesAndTorquesFollowNewtonAndEuler) {
    World world = World::make().value();
    const BodyId body = world.add_body(cylinder()).value();
    for (int step = 0; step < 128; ++step) {
        // 200 N through the centre of mass and 79.2 N m about z, for one second.
        ASSERT_TRUE(
            world.add_force(body, {200.0, 0.0, 0.0}, world.state(body).value().position_m).has_value());
        ASSERT_TRUE(world.add_torque(body, {0.0, 0.0, 79.2}).has_value());
        ASSERT_TRUE(world.step(k_step_s).has_value());
    }
    const BodyState state = world.state(body).value();
    EXPECT_NEAR(state.velocity_m_s.x, 2.0, 1e-4);           // F t / m
    EXPECT_NEAR(state.angular_velocity_rad_s.z, 2.0, 1e-3); // τ t / I
    EXPECT_NEAR(state.position_m.x, 1.0, 0.02);             // ½ a t², to the integrator's first order
}

TEST(PhysicsWorld, AForceOffTheCentreOfMassAlsoTurnsTheBody) {
    World world = World::make().value();
    const BodyId body = world.add_body(cylinder()).value();
    // 100 N along y applied 1 m ahead of the centre: 100 N m about z.
    ASSERT_TRUE(world.add_force(body, {0.0, 100.0, 0.0}, {1.0, 0.0, 0.0}).has_value());
    ASSERT_TRUE(world.step(k_step_s).has_value());
    const BodyState state = world.state(body).value();
    EXPECT_NEAR(state.velocity_m_s.y, 100.0 / 100.0 * k_step_s, 1e-7);
    EXPECT_NEAR(state.angular_velocity_rad_s.z, 100.0 / 39.6 * k_step_s, 1e-7);
    // A force lasts one step.
    ASSERT_TRUE(world.step(k_step_s).has_value());
    EXPECT_NEAR(world.state(body).value().velocity_m_s.y, k_step_s, 1e-7);
}

TEST(PhysicsWorld, ABodyDropsOntoTheGroundReportsTheImpactAndComesToRest) {
    World world = World::make().value();
    const BodyId ground = world.add_ground({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, 0.8).value();
    BodyDescription description = cylinder();
    // Standing on its end (its x axis turned to point up), its lowest point 2 m above the ground.
    description.state.orientation = helios::math::from_axis_angle({0.0, 1.0, 0.0}, -0.5 * std::numbers::pi);
    description.state.position_m = {0.0, 0.0, 3.0};
    const BodyId body = world.add_body(description).value();
    world.set_gravity({0.0, 0.0, -1.62}); // the Moon

    double impact_speed_m_s = 0.0;
    int steps_to_contact = 0;
    for (int step = 0; step < 2048; ++step) {
        ASSERT_TRUE(world.step(k_step_s).has_value());
        for (const helios::physics::Contact& contact : world.contacts_begun()) {
            EXPECT_TRUE((contact.first == body && contact.second == ground)
                        || (contact.first == ground && contact.second == body));
            if (impact_speed_m_s == 0.0) {
                impact_speed_m_s = contact.closing_speed_m_s;
                steps_to_contact = step;
            }
        }
    }
    // Free fall from 2 m at 1.62 m/s²: 2.55 m/s after 1.57 s.
    EXPECT_NEAR(impact_speed_m_s, std::sqrt(2.0 * 1.62 * 2.0), 0.1);
    EXPECT_NEAR(steps_to_contact * k_step_s, std::sqrt(2.0 * 2.0 / 1.62), 0.05);
    const BodyState state = world.state(body).value();
    EXPECT_TRUE(world.touching(body));
    EXPECT_NEAR(state.position_m.z, 1.0, 0.03); // still standing
    EXPECT_NEAR(rotate(state.orientation, {1.0, 0.0, 0.0}).z, 1.0, 1e-3);
    EXPECT_LT(norm(state.velocity_m_s), 0.01);
    EXPECT_LT(norm(state.angular_velocity_rad_s), 0.01);
}

TEST(PhysicsWorld, StateAndMassCanBeSetFromOutside) {
    World world = World::make().value();
    const BodyId body = world.add_body(cylinder()).value();
    const BodyState placed{.position_m = {10.0, -4.0, 2.0},
                           .orientation = helios::math::from_axis_angle({0.0, 1.0, 0.0}, 0.3),
                           .velocity_m_s = {1.0, 2.0, 3.0},
                           .angular_velocity_rad_s = {0.0, 0.1, 0.0}};
    ASSERT_TRUE(world.set_state(body, placed).has_value());
    const BodyState read = world.state(body).value();
    EXPECT_NEAR(norm(read.position_m - placed.position_m), 0.0, 1e-5);
    EXPECT_NEAR(norm(read.velocity_m_s - placed.velocity_m_s), 0.0, 1e-6);
    EXPECT_NEAR(read.orientation.y, placed.orientation.y, 1e-6);
    // Half the mass: the same force gives twice the acceleration.
    ASSERT_TRUE(world.set_mass(body, 50.0, diagonal(6.25, 19.8, 19.8)).has_value());
    ASSERT_TRUE(world.add_force(body, {100.0, 0.0, 0.0}, placed.position_m).has_value());
    ASSERT_TRUE(world.step(k_step_s).has_value());
    EXPECT_NEAR(world.state(body).value().velocity_m_s.x, 1.0 + 2.0 * k_step_s, 1e-6);
}

// The inside of a cabin: boxes that do not move, something that does, and rays to feel for them.
TEST(PhysicsWorld, BoxesStopASphereAndRaysFindThem) {
    World world = World::make().value();
    const BodyId floor =
        world.add_static_box({.centre_m = {0.0, 0.0, -0.5}, .half_extents_m = {5.0, 5.0, 0.5}}).value();
    // A wall turned a quarter about z: its thin side is its own y, which then lies along x.
    const BodyId wall =
        world
            .add_static_box(
                {.centre_m = {3.0, 0.0, 1.0},
                 .half_extents_m = {5.0, 0.1, 1.0},
                 .orientation = helios::math::from_axis_angle({0.0, 0.0, 1.0}, 0.5 * std::numbers::pi)})
            .value();
    const BodyId ball = world
                            .add_body({.pieces = {ShapePiece{.radius_m = 0.3}},
                                       .mass_kg = 80.0,
                                       .inertia_kg_m2 = diagonal(2.88, 2.88, 2.88),
                                       .state = {.position_m = {0.0, 0.0, 2.0}}})
                            .value();
    world.set_gravity({0.0, 0.0, -9.81});
    run(world, 256);
    const BodyState rested = world.state(ball).value();
    EXPECT_NEAR(rested.position_m.z, 0.3, 0.02);
    EXPECT_LT(norm(rested.velocity_m_s), 0.02);
    EXPECT_TRUE(world.touching(ball));

    const auto down = world.cast_ray({1.0, 0.0, 2.0}, {0.0, 0.0, -1.0}, 5.0);
    ASSERT_TRUE(down.has_value());
    EXPECT_EQ(down->body, floor);
    EXPECT_NEAR(down->distance_m, 2.0, 1e-4);
    EXPECT_NEAR(norm(down->point_m - Vector3{1.0, 0.0, 0.0}), 0.0, 1e-4);
    EXPECT_NEAR(norm(down->normal - Vector3{0.0, 0.0, 1.0}), 0.0, 1e-4);

    // From inside the ball the ball itself is in the way, unless it is passed through.
    const Vector3 centre_m = rested.position_m;
    const auto blocked = world.cast_ray(centre_m, {1.0, 0.0, 0.0}, 5.0);
    ASSERT_TRUE(blocked.has_value());
    EXPECT_EQ(blocked->body, ball);
    const auto ahead = world.cast_ray(centre_m, {1.0, 0.0, 0.0}, 5.0, ball);
    ASSERT_TRUE(ahead.has_value());
    EXPECT_EQ(ahead->body, wall);
    EXPECT_NEAR(ahead->distance_m, 2.9 - centre_m.x, 1e-3);
    EXPECT_NEAR(norm(ahead->normal - Vector3{-1.0, 0.0, 0.0}), 0.0, 1e-4);
    // Out of reach, or with a direction that is not one, there is nothing.
    EXPECT_FALSE(world.cast_ray(centre_m, {1.0, 0.0, 0.0}, 1.0, ball).has_value());
    EXPECT_FALSE(world.cast_ray(centre_m, {2.0, 0.0, 0.0}, 5.0, ball).has_value());
    EXPECT_FALSE(world.cast_ray(centre_m, {0.0, 0.0, 1.0}, 50.0, ball).has_value());

    EXPECT_EQ(world.add_static_box({.centre_m = {}, .half_extents_m = {1.0, 0.0, 1.0}}).error().code,
              ErrorCode::InvalidArgument);

    // A box that moves: dropped flat on the floor it comes to rest on its face.
    const BodyId crate = world
                             .add_body({.pieces = {ShapePiece{.box_half_extents_m = {0.3, 0.2, 0.1}}},
                                        .mass_kg = 10.0,
                                        .inertia_kg_m2 = diagonal(0.17, 0.33, 0.43),
                                        .state = {.position_m = {-2.0, 0.0, 1.0}}})
                             .value();
    run(world, 256);
    EXPECT_NEAR(world.state(crate).value().position_m.z, 0.1, 0.02);
    EXPECT_LT(norm(world.state(crate).value().velocity_m_s), 0.02);
    const BodyDescription flat_box{.pieces = {ShapePiece{.box_half_extents_m = {0.3, -0.2, 0.1}}},
                                   .mass_kg = 1.0,
                                   .inertia_kg_m2 = diagonal(1.0, 1.0, 1.0)};
    EXPECT_EQ(world.add_body(flat_box).error().code, ErrorCode::OutOfRange);
}

TEST(PhysicsWorld, RejectsInvalidBodiesAndUnknownIds) {
    World world = World::make().value();
    BodyDescription massless = cylinder();
    massless.mass_kg = 0.0;
    EXPECT_EQ(world.add_body(massless).error().code, ErrorCode::OutOfRange);
    BodyDescription flat = cylinder();
    flat.pieces.front().radius_m = 0.0;
    EXPECT_EQ(world.add_body(flat).error().code, ErrorCode::OutOfRange);
    BodyDescription lost = cylinder();
    lost.state.position_m.x = std::nan("");
    EXPECT_EQ(world.add_body(lost).error().code, ErrorCode::NotFinite);
    EXPECT_EQ(world.add_ground({}, {0.0, 0.0, 2.0}, 0.5).error().code, ErrorCode::InvalidArgument);

    const BodyId unknown{12345};
    EXPECT_EQ(world.state(unknown).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(world.set_state(unknown, {}).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(world.add_force(unknown, {}, {}).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(world.add_torque(unknown, {}).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(world.set_mass(unknown, 1.0, diagonal(1.0, 1.0, 1.0)).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(world.remove(unknown).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(world.step(0.0).error().code, ErrorCode::OutOfRange);

    // A body with no shape is allowed, and a removed body is gone.
    BodyDescription point = cylinder();
    point.pieces.clear();
    const BodyId body = world.add_body(point).value();
    ASSERT_TRUE(world.remove(body).has_value());
    EXPECT_EQ(world.state(body).error().code, ErrorCode::InvalidArgument);
    const BodyId ground = world.add_ground({}, {0.0, 0.0, 1.0}, 0.5).value();
    EXPECT_EQ(world.set_mass(ground, 1.0, diagonal(1.0, 1.0, 1.0)).error().code, ErrorCode::InvalidArgument);
}

} // namespace
