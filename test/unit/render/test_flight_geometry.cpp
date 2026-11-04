#include "helios/render/flight_geometry.hpp"
#include "helios/render/mesh.hpp"
#include "helios/render/stroke_font.hpp"
#include "helios/vessel/part_datasheet.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <gtest/gtest.h>
#include <memory>
#include <numbers>
#include <span>
#include <string_view>
#include <utility>

namespace {

using helios::math::Vector3;
using helios::render::ControlHandle;
using helios::render::FlightGeometry;
using helios::render::Float3;
using helios::render::HeadPose;
using helios::render::Mesh;
using helios::render::ViewPoint;
using helios::sim::SceneSnapshot;
using helios::sim::VesselView;
using helios::vessel::Control;
using helios::vessel::Indicator;

constexpr double k_moon_radius_m = 1'737'400.0;
constexpr double k_height_m = 12.0;

[[nodiscard]] Vector3 to_double(const Float3& vector) {
    return {vector.x, vector.y, vector.z};
}

// Every triangle of a closed mesh faces the way its vertices' normals do.
void expect_outward(const Mesh& mesh) {
    ASSERT_EQ(mesh.indices.size() % 3, 0U);
    for (std::size_t triangle = 0; triangle < mesh.indices.size(); triangle += 3) {
        const helios::render::MeshVertex& a = mesh.vertices.at(mesh.indices[triangle]);
        const helios::render::MeshVertex& b = mesh.vertices.at(mesh.indices[triangle + 1]);
        const helios::render::MeshVertex& c = mesh.vertices.at(mesh.indices[triangle + 2]);
        const Vector3 face = cross(to_double(b.position) - to_double(a.position),
                                   to_double(c.position) - to_double(a.position));
        EXPECT_GT(dot(face, to_double(a.normal) + to_double(b.normal) + to_double(c.normal)), 0.0)
            << "triangle " << triangle / 3;
        EXPECT_NEAR(norm(to_double(a.normal)), 1.0, 1e-6);
    }
}

TEST(Mesh, TheBoxAndTheCylinderAreClosedAndFaceOutwards) {
    const Mesh box = helios::render::make_box();
    EXPECT_EQ(box.vertices.size(), 24U);
    EXPECT_EQ(box.indices.size(), 36U);
    expect_outward(box);
    const Mesh cylinder = helios::render::make_cylinder(16);
    EXPECT_EQ(cylinder.indices.size(), 3U * (2U * 16U + 2U * 16U));
    expect_outward(cylinder);
    for (const helios::render::MeshVertex& vertex : cylinder.vertices) {
        EXPECT_NEAR(std::abs(vertex.position.x), 0.5F, 1e-6F);
        EXPECT_LE(std::hypot(vertex.position.y, vertex.position.z), 1.0F + 1e-6F);
    }
}

// A pod with one seat, a throttle lever on a console, and on the panel ahead a dial and a
// switch under a cover.
[[nodiscard]] std::shared_ptr<const helios::vessel::PartDatasheet> make_pod() {
    helios::vessel::PartDatasheet pod;
    pod.id = "test.pod";
    pod.shape = helios::vessel::Cylinder{.radius_m = 1.0, .length_m = 2.0};
    helios::vessel::Cockpit cockpit;
    cockpit.eye_m = {0.2, 0.0, 0.1};
    cockpit.boxes.push_back({.centre_m = {0.6, 0.0, -0.3}, .size_m = {0.04, 1.0, 0.4}});
    cockpit.boxes.push_back({.centre_m = {0.7, 0.0, 0.3}, .size_m = {0.04, 1.0, 0.8}, .glass = true});
    cockpit.controls.push_back(
        Control{.id = "throttle",
                .type = Control::Type::Axis,
                .label = "THROTTLE",
                .place = {.position_m = {0.3, 0.4, -0.4}, .facing = {0.0, 0.0, 1.0}, .up = {1.0, 0.0, 0.0}},
                .signal = "engine/throttle"});
    Control cut{.id = "cut",
                .type = Control::Type::Switch,
                .label = "CUT",
                .place = {.position_m = {0.58, -0.3, -0.2}},
                .signal = "engine/throttle",
                .positions = {{.settings = {{.signal = "engine/throttle", .value = 0.0}}},
                              {.settings = {{.signal = "engine/throttle", .value = 1.0}}}},
                .guard = helios::vessel::Guard{}};
    cockpit.controls.push_back(std::move(cut));
    cockpit.indicators.push_back(Indicator{.id = "mass",
                                           .type = Indicator::Type::Gauge,
                                           .label = "MASS",
                                           .place = {.position_m = {0.58, 0.0, -0.2}},
                                           .signal = "vessel/mass_kg",
                                           .maximum = 6000.0,
                                           .scale = 0.001,
                                           .unit = "t"});
    EXPECT_TRUE(helios::vessel::resolve_looks(cockpit, {}).has_value()); // the built-in looks
    pod.cockpit = std::move(cockpit);
    return std::make_shared<const helios::vessel::PartDatasheet>(std::move(pod));
}

// The pod on a tank, `k_height_m` above a body the size of the Moon, nose along the universe's
// +x (which is also straight up from the ground there). The star is far along +y.
[[nodiscard]] SceneSnapshot make_snapshot(double throttle) {
    SceneSnapshot snapshot;
    snapshot.focus = helios::sim::Focus::vessel({0});
    snapshot.bodies.push_back(
        helios::sim::BodyView{.id = {0}, .name = "Sun", .position_m = {0.0, 1.5e11, 0.0}, .radius_m = 7e8});
    snapshot.bodies.push_back(helios::sim::BodyView{.id = {1},
                                                    .name = "Moon",
                                                    .position_m = {-(k_moon_radius_m + k_height_m), 0.0, 0.0},
                                                    .radius_m = k_moon_radius_m,
                                                    .domain_parent = helios::bodies::BodyId{0}});
    helios::vessel::PartDatasheet tank;
    tank.id = "test.tank";
    tank.shape = helios::vessel::Cylinder{.radius_m = 0.5, .length_m = 3.0};
    VesselView vessel;
    vessel.id = {0};
    vessel.name = "Test";
    vessel.domain = {1};
    vessel.signals.push_back(
        {.name = "engine/throttle", .command = true, .value = throttle, .minimum = 0.0, .maximum = 1.0});
    vessel.signals.push_back({.name = "vessel/mass_kg", .unit = "kg", .value = 4500.0});
    vessel.parts.resize(2);
    vessel.parts[0].name = "pod";
    vessel.parts[0].position_m = {1.0, 0.0, 0.0};
    vessel.parts[0].datasheet = make_pod();
    vessel.parts[1].name = "tank";
    vessel.parts[1].position_m = {-1.5, 0.0, 0.0};
    vessel.parts[1].datasheet = std::make_shared<const helios::vessel::PartDatasheet>(std::move(tank));
    snapshot.vessels.push_back(std::move(vessel));
    return snapshot;
}

TEST(FlightView, TheSeatLooksAlongTheNoseAndTheHeadTurns) {
    const SceneSnapshot snapshot = make_snapshot(0.0);
    const VesselView& vessel = snapshot.vessels.front();
    ASSERT_EQ(helios::render::seat_part(vessel), 0U);
    const ViewPoint ahead = helios::render::seat_view_point(vessel, HeadPose{}).value();
    // The eye: the part's place in the vessel plus the eye's place in the part.
    EXPECT_EQ(ahead.position_from_focus_m, (Vector3{1.2, 0.0, 0.1}));
    // The camera looks along its −z; its x is to the right, which is the seat's −y.
    EXPECT_NEAR(norm(ahead.universe_to_camera * Vector3{1.0, 0.0, 0.0} - Vector3{0.0, 0.0, -1.0}), 0.0,
                1e-12);
    EXPECT_NEAR(norm(ahead.universe_to_camera * Vector3{0.0, -1.0, 0.0} - Vector3{1.0, 0.0, 0.0}), 0.0,
                1e-12);
    EXPECT_NEAR(norm(ahead.universe_to_camera * Vector3{0.0, 0.0, 1.0} - Vector3{0.0, 1.0, 0.0}), 0.0, 1e-12);

    HeadPose head;
    head.turn(0.5 * std::numbers::pi, 0.0); // to the left
    const ViewPoint left = helios::render::seat_view_point(vessel, head).value();
    EXPECT_NEAR(norm(left.universe_to_camera * Vector3{0.0, 1.0, 0.0} - Vector3{0.0, 0.0, -1.0}), 0.0, 1e-12);
    head.turn(10.0, 10.0);
    EXPECT_EQ(head.yaw_rad, HeadPose::k_max_yaw_rad);
    EXPECT_EQ(head.pitch_rad, HeadPose::k_max_pitch_rad);

    // No seat, no view from it.
    VesselView bare = vessel;
    bare.parts.erase(bare.parts.begin());
    EXPECT_FALSE(helios::render::seat_view_point(bare, HeadPose{}).has_value());
    EXPECT_FALSE(helios::render::seat_to_vessel(bare).has_value());
}

TEST(FlightView, AHeadsetMovesAndTurnsTheHeadAboutTheSeat) {
    using helios::render::TrackedPose;
    const SceneSnapshot snapshot = make_snapshot(0.0);
    const VesselView& vessel = snapshot.vessels.front();
    // A head that has not moved sees what the seat does.
    const ViewPoint seat = helios::render::seat_view_point(vessel, HeadPose{}).value();
    const ViewPoint still = helios::render::seat_view_point(vessel, TrackedPose{}).value();
    EXPECT_EQ(still.position_from_focus_m, seat.position_from_focus_m);
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            EXPECT_NEAR(still.universe_to_camera(row, column), seat.universe_to_camera(row, column), 1e-12);
        }
    }
    // Leaning 10 cm forward and 5 cm to the left, looking to the left: the seat's axes are the
    // vessel's here.
    const TrackedPose leaning{.position_m = {0.1, 0.05, 0.0},
                              .orientation =
                                  helios::math::from_axis_angle({0.0, 0.0, 1.0}, 0.5 * std::numbers::pi)};
    const ViewPoint moved = helios::render::seat_view_point(vessel, leaning).value();
    EXPECT_NEAR(norm(moved.position_from_focus_m - Vector3{1.3, 0.05, 0.1}), 0.0, 1e-12);
    EXPECT_NEAR(norm(moved.universe_to_camera * Vector3{0.0, 1.0, 0.0} - Vector3{0.0, 0.0, -1.0}), 0.0,
                1e-12);
    EXPECT_NEAR(norm(moved.universe_to_camera * Vector3{0.0, 0.0, 1.0} - Vector3{0.0, 1.0, 0.0}), 0.0, 1e-12);

    // Out of the seat the head is tracked about the pilot's eyes, in the axes the body faces.
    const helios::sim::PilotView pilot{.position_m = {0.5, 0.25, -0.5}, .vessel = {0}, .seated = false};
    const helios::math::Quaternion facing_left =
        helios::math::from_axis_angle({0.0, 0.0, 1.0}, 0.5 * std::numbers::pi);
    const ViewPoint afoot = helios::render::pilot_view_point(vessel, pilot, facing_left, TrackedPose{});
    const ViewPoint plain = helios::render::pilot_view_point(vessel, pilot, facing_left);
    EXPECT_NEAR(norm(afoot.position_from_focus_m - plain.position_from_focus_m), 0.0, 1e-12);
    const ViewPoint stepped = helios::render::pilot_view_point(vessel, pilot, facing_left,
                                                               TrackedPose{.position_m = {0.2, 0.0, 0.0}});
    // Forward for a body facing left is the vessel's +y.
    EXPECT_NEAR(norm(stepped.position_from_focus_m - plain.position_from_focus_m - Vector3{0.0, 0.2, 0.0}),
                0.0, 1e-12);

    // On foot outside the look is in the ground's axes, not the suit's: with the ground a
    // quarter turn about z from the universe, looking along its +x is looking along the
    // universe's +y, however the suit is turned. The eyes stay where the suit has them.
    helios::sim::PilotView walker = pilot;
    walker.ground = facing_left;
    const ViewPoint on_foot = helios::render::pilot_view_point(vessel, walker, helios::math::Quaternion{});
    EXPECT_NEAR(norm(on_foot.universe_to_camera * Vector3{0.0, 1.0, 0.0} - Vector3{0.0, 0.0, -1.0}), 0.0,
                1e-12);
    EXPECT_NEAR(norm(on_foot.position_from_focus_m - plain.position_from_focus_m), 0.0, 1e-12);
    const ViewPoint on_foot_stepped = helios::render::pilot_view_point(
        vessel, walker, helios::math::Quaternion{}, TrackedPose{.position_m = {0.2, 0.0, 0.0}});
    EXPECT_NEAR(
        norm(on_foot_stepped.position_from_focus_m - plain.position_from_focus_m - Vector3{0.0, 0.2, 0.0}),
        0.0, 1e-12);

    VesselView bare = vessel;
    bare.parts.erase(bare.parts.begin());
    EXPECT_FALSE(helios::render::seat_view_point(bare, TrackedPose{}).has_value());
}

TEST(FlightView, TheChaseCameraKeepsTheHorizonLevel) {
    const SceneSnapshot snapshot = make_snapshot(0.0);
    const helios::render::OrbitCamera camera{.yaw_rad = 0.7, .pitch_rad = 0.3, .distance_m = 20.0};
    const ViewPoint view = helios::render::chase_view_point(snapshot, snapshot.vessels.front(), camera);
    EXPECT_NEAR(norm(view.position_from_focus_m), 20.0, 1e-9);
    // Up from the ground is +x here: the camera is above the vessel, and its right is level.
    EXPECT_NEAR(view.position_from_focus_m.x, 20.0 * std::sin(0.3), 1e-9);
    const Vector3 right = transpose(view.universe_to_camera) * Vector3{1.0, 0.0, 0.0};
    EXPECT_NEAR(right.x, 0.0, 1e-12);
    // It looks at the vessel.
    const Vector3 to_vessel = view.universe_to_camera * (-1.0 * view.position_from_focus_m);
    EXPECT_NEAR(to_vessel.z, -20.0, 1e-9);
}

TEST(FlightView, TheGroundPatchLiesOnTheBodyAndReplacesItsSphere) {
    const SceneSnapshot snapshot = make_snapshot(0.0);
    const ViewPoint view{.position_from_focus_m = {}, .universe_to_camera = {}};
    const FlightGeometry geometry = helios::render::build_flight_geometry(snapshot, 0, view, false);
    ASSERT_TRUE(geometry.terrain.has_value());
    const Mesh& ground = geometry.terrain->mesh;
    // The first vertex is straight below the camera, at the surface.
    EXPECT_FLOAT_EQ(ground.vertices.front().position.x, static_cast<float>(-k_height_m));
    EXPECT_FLOAT_EQ(ground.vertices.front().normal.x, 1.0F);
    const Vector3 centre_m = snapshot.bodies[1].position_m;
    double furthest_m = 0.0;
    for (const helios::render::MeshVertex& vertex : ground.vertices) {
        const Vector3 position_m = to_double(vertex.position);
        // Rationale for the tolerance: a float carries about 7 digits of the distance from the
        // camera, which is up to the 6.5 km of the horizon.
        EXPECT_NEAR(norm(position_m - centre_m), k_moon_radius_m, 0.05);
        furthest_m = std::max(furthest_m, norm(position_m));
    }
    // It reaches the horizon: √(2 R h) away.
    EXPECT_GT(furthest_m, std::sqrt(2.0 * k_moon_radius_m * k_height_m));
    expect_outward(ground);
    // Cells of 20 m at 12 m up; the point below the camera is within half a cell of a grid line
    // crossing.
    EXPECT_DOUBLE_EQ(geometry.terrain->cell_m, 20.0);
    EXPECT_LE(std::abs(ground.vertices.front().u), 0.5F);
    EXPECT_LE(std::abs(ground.vertices.front().v), 0.5F);
    // The Moon's sphere is not drawn as well; the Sun's is.
    ASSERT_EQ(geometry.scene.bodies.size(), 1U);
    EXPECT_EQ(geometry.scene.bodies.front().body_index, 0U);
    EXPECT_TRUE(geometry.scene.lines.empty());
    EXPECT_NEAR(geometry.sun_direction.y, 1.0F, 1e-6F);

    // From far away the sphere will do.
    SceneSnapshot distant = snapshot;
    distant.bodies[1].position_m = {-3.0 * k_moon_radius_m, 0.0, 0.0};
    const FlightGeometry from_afar = helios::render::build_flight_geometry(distant, 0, view, false);
    EXPECT_FALSE(from_afar.terrain.has_value());
    EXPECT_EQ(from_afar.scene.bodies.size(), 2U);
}

TEST(FlightView, FromOutsideThePartsAreDrawnAndFromTheSeatTheCabin) {
    const SceneSnapshot snapshot = make_snapshot(0.0);
    const ViewPoint outside{.position_from_focus_m = {0.0, -30.0, 0.0}, .universe_to_camera = {}};
    const FlightGeometry seen = helios::render::build_flight_geometry(snapshot, 0, outside, false);
    ASSERT_EQ(seen.solids.size(), 2U); // the pod and the tank
    EXPECT_TRUE(seen.controls.empty());
    const helios::render::SolidInstance& tank = seen.solids[1];
    EXPECT_EQ(tank.shape, helios::render::SolidShape::Cylinder);
    // Its length along x, its radius across, and its centre relative to the camera.
    EXPECT_FLOAT_EQ(tank.model[0], 3.0F);
    EXPECT_FLOAT_EQ(tank.model[5], 0.5F);
    EXPECT_FLOAT_EQ(tank.model[12], -1.5F);
    EXPECT_FLOAT_EQ(tank.model[13], 30.0F);

    const ViewPoint seat = helios::render::seat_view_point(snapshot.vessels.front(), HeadPose{}).value();
    const FlightGeometry inside = helios::render::build_flight_geometry(snapshot, 0, seat, true);
    ASSERT_EQ(inside.controls.size(), 2U); // the lever, and the cover over the switch
    // The tank is still there; the pod's hull is replaced by its interior.
    EXPECT_EQ(std::ranges::count(inside.solids, helios::render::SolidShape::Cylinder,
                                 &helios::render::SolidInstance::shape),
              2); // the tank and the dial's face
    EXPECT_GT(inside.solids.size(), 8U);
}

TEST(FlightView, ThePilotAfootSeesFromWhereTheBodyIsAndTheHandShows) {
    SceneSnapshot snapshot = make_snapshot(0.0);
    const helios::sim::PilotView pilot{.position_m = {0.5, 0.25, -0.5},
                                       .grip_m = {1.0, 0.25, -0.5},
                                       .vessel = {0},
                                       .seated = false,
                                       .grabbing = true};
    // Eyes level with the vessel's axes: 0.2 m above the body's centre, looking along the nose.
    const ViewPoint level =
        helios::render::pilot_view_point(snapshot.vessels.front(), pilot, helios::math::Quaternion{});
    EXPECT_NEAR(norm(level.position_from_focus_m - Vector3{0.5, 0.25, -0.3}), 0.0, 1e-12);
    EXPECT_NEAR(norm(level.universe_to_camera * Vector3{1.0, 0.0, 0.0} - Vector3{0.0, 0.0, -1.0}), 0.0,
                1e-12);
    EXPECT_NEAR(norm(level.universe_to_camera * Vector3{0.0, 1.0, 0.0} - Vector3{-1.0, 0.0, 0.0}), 0.0,
                1e-12);
    // Head over heels: the eyes are then below the centre, and the picture upside down.
    const ViewPoint inverted = helios::render::pilot_view_point(
        snapshot.vessels.front(), pilot, helios::math::from_axis_angle({1.0, 0.0, 0.0}, std::numbers::pi));
    EXPECT_NEAR(inverted.position_from_focus_m.z, -0.7, 1e-12);
    EXPECT_NEAR((inverted.universe_to_camera * Vector3{0.0, 0.0, 1.0}).y, -1.0, 1e-12);

    // The hand is drawn where it holds, from inside only, and only while it holds.
    const std::size_t without = helios::render::build_flight_geometry(snapshot, 0, level, true).solids.size();
    snapshot.pilot = pilot;
    const FlightGeometry holding = helios::render::build_flight_geometry(snapshot, 0, level, true);
    ASSERT_EQ(holding.solids.size(), without + 1);
    EXPECT_NEAR(holding.solids.back().model[12], 0.5F, 1e-6F); // the grip, half a metre ahead of the eyes
    EXPECT_EQ(helios::render::build_flight_geometry(snapshot, 0, level, false).solids.size(), 2U);
    snapshot.pilot->grabbing = false;
    EXPECT_EQ(helios::render::build_flight_geometry(snapshot, 0, level, true).solids.size(), without);

    // Loose items are drawn where they are, turned as they are.
    snapshot.items.push_back(
        {.name = "crate",
         .position_m = {0.9, 0.25, -0.3},
         .orientation = helios::math::from_axis_angle({0.0, 0.0, 1.0}, 0.5 * std::numbers::pi),
         .size_m = {0.4, 0.2, 0.1},
         .colour = {1.0, 0.0, 0.0}});
    const FlightGeometry loose = helios::render::build_flight_geometry(snapshot, 0, level, true);
    ASSERT_EQ(loose.solids.size(), without + 1);
    const helios::render::SolidInstance& crate = loose.solids.back();
    EXPECT_NEAR(crate.model[12], 0.4F, 1e-6F); // straight ahead of the eyes
    EXPECT_NEAR(crate.model[13], 0.0F, 1e-6F);
    EXPECT_NEAR(crate.model[1], 0.4F, 1e-6F); // its long side now lies along the vessel's y
    EXPECT_EQ(crate.abgr, helios::render::rgba(255, 0, 0));
    // One a hand holds is drawn where it is too: the hand holds it where it took hold of it.
    snapshot.items.front().held = true;
    const FlightGeometry in_hand = helios::render::build_flight_geometry(snapshot, 0, level, true);
    EXPECT_NEAR(in_hand.solids.back().model[12], 0.4F, 1e-6F);
    // From outside nothing of the cabin is drawn.
    EXPECT_EQ(helios::render::build_flight_geometry(snapshot, 0, level, false).solids.size(), 2U);
}

TEST(FlightView, ControlsShowTheirSignalsAndCanBeHandled) {
    const auto handles_at = [](double throttle) {
        const SceneSnapshot snapshot = make_snapshot(throttle);
        const ViewPoint seat = helios::render::seat_view_point(snapshot.vessels.front(), HeadPose{}).value();
        return helios::render::build_flight_geometry(snapshot, 0, seat, true).controls;
    };
    const auto closed = handles_at(0.0);
    const auto open = handles_at(1.0);
    const ControlHandle& lever = open[0];
    EXPECT_EQ(lever.control, 0U);
    EXPECT_TRUE(lever.live);
    // The handle moves its whole travel, forward (the camera's own axes are the universe's
    // here, relative to the eye).
    EXPECT_NEAR(lever.centre.x - closed[0].centre.x, 0.3F, 1e-6F);
    EXPECT_NEAR(lever.travel.x, 0.3F, 1e-6F);
    EXPECT_NEAR(closed[0].centre.x, 0.3F - 0.15F, 1e-6F);
    EXPECT_NEAR(closed[0].centre.y, 0.4F, 1e-6F);

    // A ray at the handle finds it; one at the dial finds nothing to take hold of.
    const auto towards = [](const Float3& point) {
        const float length = std::sqrt((point.x * point.x) + (point.y * point.y) + (point.z * point.z));
        return Float3{point.x / length, point.y / length, point.z / length};
    };
    EXPECT_EQ(helios::render::pick_control(open, towards(lever.centre)).value().handle, 0U);
    EXPECT_FALSE(helios::render::pick_control(open, towards(Float3{0.58F, 0.0F, -0.2F})).has_value());
    EXPECT_FALSE(helios::render::pick_control(open, Float3{-1.0F, 0.0F, 0.0F}).has_value());

    // A vessel without the signal: the control is there, but dead and out of reach.
    SceneSnapshot unpowered = make_snapshot(0.0);
    unpowered.vessels.front().signals.clear();
    const ViewPoint seat = helios::render::seat_view_point(unpowered.vessels.front(), HeadPose{}).value();
    const auto dead = helios::render::build_flight_geometry(unpowered, 0, seat, true).controls;
    EXPECT_FALSE(dead[0].live);
    EXPECT_FALSE(helios::render::pick_control(dead, towards(dead[0].centre)).has_value());
}

// A guarded control (D43): the cover is in the way until it is lifted.
TEST(FlightView, AGuardHidesItsControlUntilLifted) {
    SceneSnapshot snapshot = make_snapshot(1.0);
    const ViewPoint seat = helios::render::seat_view_point(snapshot.vessels.front(), HeadPose{}).value();
    const auto towards = [](const Float3& point) {
        const float length = std::sqrt((point.x * point.x) + (point.y * point.y) + (point.z * point.z));
        return Float3{point.x / length, point.y / length, point.z / length};
    };
    const auto shut = helios::render::build_flight_geometry(snapshot, 0, seat, true).controls;
    ASSERT_EQ(shut.size(), 2U);
    const ControlHandle& cover = shut[1];
    EXPECT_TRUE(cover.guard);
    EXPECT_EQ(cover.control, 1U);
    EXPECT_EQ(helios::render::pick_control(shut, towards(cover.centre)).value().handle, 1U);

    snapshot.vessels.front().parts.front().open_guards = {"cut"};
    const auto lifted = helios::render::build_flight_geometry(snapshot, 0, seat, true).controls;
    ASSERT_EQ(lifted.size(), 3U);
    const ControlHandle& toggle = lifted[2];
    EXPECT_FALSE(toggle.guard);
    EXPECT_EQ(toggle.control, 1U);
    // Lifted, the cover stands off the panel towards the pilot, out of the switch's way.
    EXPECT_LT(lifted[1].centre.x, cover.centre.x - 0.01F);
    // A click on the far side of a switch (up its travel) is to its next position.
    const Float3 above{toggle.centre.x + (0.4F * toggle.travel.x), toggle.centre.y + (0.4F * toggle.travel.y),
                       toggle.centre.z + (0.4F * toggle.travel.z)};
    const auto picked = helios::render::pick_control(std::span(&toggle, 1), towards(above));
    ASSERT_TRUE(picked.has_value());
    EXPECT_GT(picked->side, 0.0);
}

TEST(FlightView, PanelsAreLetteredWithLabelsAndReadings) {
    const SceneSnapshot snapshot = make_snapshot(1.0);
    const ViewPoint seat = helios::render::seat_view_point(snapshot.vessels.front(), HeadPose{}).value();
    const FlightGeometry geometry = helios::render::build_flight_geometry(snapshot, 0, seat, true);
    // Each stroke of lettering is two triangles: every label and reading is there.
    std::size_t strokes = 0;
    for (const std::string_view text : {"THROTTLE", "CUT", "MASS", "4.50 t"}) {
        strokes += helios::render::text_strokes(text).size();
    }
    EXPECT_EQ(geometry.text.size(), 6U * strokes);
    // A label is painted, a reading lit; a dead control's label is grey and has no reading.
    const auto count_colour = [](const std::vector<helios::render::LineVertex>& text, std::uint32_t abgr) {
        return std::ranges::count(text, abgr, &helios::render::LineVertex::abgr);
    };
    EXPECT_GT(count_colour(geometry.text, helios::render::rgba(255, 190, 90)), 0);
    SceneSnapshot unpowered = make_snapshot(0.0);
    unpowered.vessels.front().signals.clear();
    const FlightGeometry dead = helios::render::build_flight_geometry(unpowered, 0, seat, true);
    EXPECT_EQ(count_colour(dead.text, helios::render::rgba(255, 190, 90)), 0);
    EXPECT_EQ(count_colour(dead.text, helios::render::rgba(90, 90, 95)),
              static_cast<std::ptrdiff_t>(dead.text.size()));
    // From outside there is none.
    EXPECT_TRUE(helios::render::build_flight_geometry(snapshot, 0, seat, false).text.empty());
}

TEST(FlightView, ALeverFollowsThePointerAndPixelsBecomeRays) {
    const SceneSnapshot snapshot = make_snapshot(0.5);
    // Looking down at the console from above it.
    HeadPose head;
    head.turn(0.9, -0.9);
    const ViewPoint seat = helios::render::seat_view_point(snapshot.vessels.front(), head).value();
    const auto handles = helios::render::build_flight_geometry(snapshot, 0, seat, true).controls;
    const helios::render::Matrix4f view_projection =
        helios::render::multiply(helios::render::reversed_infinite_projection(1.2, 16.0 / 9.0, 0.05, false),
                                 helios::render::view_matrix(seat.universe_to_camera));
    const ControlHandle& lever = handles[0];
    const Float3 low{lever.centre.x - (0.5F * lever.travel.x), lever.centre.y - (0.5F * lever.travel.y),
                     lever.centre.z - (0.5F * lever.travel.z)};
    const Float3 high{low.x + lever.travel.x, low.y + lever.travel.y, low.z + lever.travel.z};
    const auto from = helios::render::project_to_screen(low, view_projection, 1280.0F, 720.0F).value();
    const auto to = helios::render::project_to_screen(high, view_projection, 1280.0F, 720.0F).value();
    // Dragging from one end of the travel to the other is the whole range; across it, nothing.
    EXPECT_NEAR(helios::render::travel_fraction(lever.centre, lever.travel, view_projection, 1280.0F, 720.0F,
                                                to.x - from.x, to.y - from.y),
                1.0, 1e-6);
    EXPECT_NEAR(helios::render::travel_fraction(lever.centre, lever.travel, view_projection, 1280.0F, 720.0F,
                                                -(to.y - from.y), to.x - from.x),
                0.0, 1e-6);

    // The pixel a point is drawn at gives back the ray to that point.
    const auto at = helios::render::project_to_screen(lever.centre, view_projection, 1280.0F, 720.0F).value();
    const Float3 ray =
        helios::render::ray_through_pixel(at.x, at.y, 1280.0F, 720.0F, 1.2, seat.universe_to_camera);
    EXPECT_EQ(helios::render::pick_control(handles, ray).value().handle, 0U);
    const float length = std::sqrt((lever.centre.x * lever.centre.x) + (lever.centre.y * lever.centre.y)
                                   + (lever.centre.z * lever.centre.z));
    EXPECT_NEAR(ray.x * length, lever.centre.x, 1e-4F);
    EXPECT_NEAR(ray.y * length, lever.centre.y, 1e-4F);
    EXPECT_NEAR(ray.z * length, lever.centre.z, 1e-4F);
}

// A controller's hand (D39) touches the handle its fingertips are at, moves a lever by the part of
// its movement along the travel, and is drawn with its fingertips where they are.
TEST(FlightView, AControllersHandTouchesAndMovesControls) {
    const SceneSnapshot snapshot = make_snapshot(0.5);
    const ViewPoint seat = helios::render::seat_view_point(snapshot.vessels.front(), HeadPose{}).value();
    FlightGeometry geometry = helios::render::build_flight_geometry(snapshot, 0, seat, true);
    const auto& handles = geometry.controls;
    const ControlHandle& lever = handles[0];
    const Vector3 centre = to_double(lever.centre);
    const Vector3 travel = to_double(lever.travel);
    EXPECT_EQ(helios::render::touch_control(handles, centre, 0.03).value().handle, 0U);
    // Out of reach: beyond the handle's radius and the reach.
    const Vector3 away = centre + Vector3{0.0, static_cast<double>(lever.radius_m) + 0.05, 0.0};
    const auto far = helios::render::touch_control(handles, away, 0.03);
    EXPECT_TRUE(!far.has_value() || far->handle != 0U);

    EXPECT_NEAR(helios::render::travel_fraction(lever.travel, travel), 1.0, 1e-9);
    EXPECT_NEAR(helios::render::travel_fraction(lever.travel, -0.25 * travel), -0.25, 1e-9);
    const Vector3 across = cross(travel, Vector3{1.0, 0.0, 0.0});
    EXPECT_NEAR(helios::render::travel_fraction(lever.travel, across), 0.0, 1e-9);

    const std::size_t solids = geometry.solids.size();
    const helios::render::ControllerHand hand{.position_m = centre, .hand_to_space = {}, .closed = false};
    helios::render::add_controller_hands(geometry, std::span(&hand, 1));
    ASSERT_EQ(geometry.solids.size(), solids + 1);
    // The block reaches back from the fingertips along the hand's forward (the universe's x).
    const helios::render::Matrix4f& model = geometry.solids.back().model;
    EXPECT_NEAR(static_cast<double>(model[12]) + 0.5 * static_cast<double>(model[0]), centre.x, 1e-5);
    EXPECT_NEAR(static_cast<double>(model[13]), centre.y, 1e-5);
    EXPECT_GT(model[0], 0.1F);
}

// The suit's tether (D36): a reel where it is made fast, and the line straight from the suit to
// it, as long as the two are apart whatever the length of the tether: the reel takes up the rest.
TEST(FlightGeometry, DrawsTheTetherFromTheSuitToItsReel) {
    const auto drawn = [](double apart_m) {
        SceneSnapshot snapshot;
        snapshot.vessels.push_back(VesselView{.id = helios::sim::VesselId{0}, .name = "suit"});
        // The host's centre of mass 2 m off, the line made fast a further metre on.
        snapshot.vessels.push_back(VesselView{
            .id = helios::sim::VesselId{1}, .name = "habitat", .position_m = {apart_m - 1.0, 0.0, 0.0}});
        snapshot.pilot =
            helios::sim::PilotView{.tether = helios::sim::TetherView{.point_m = {1.0, 0.0, 0.0},
                                                                     .length_m = 10.0,
                                                                     .apart_m = apart_m,
                                                                     .vessel = helios::sim::VesselId{1}},
                                   .vessel = helios::sim::VesselId{0},
                                   .outside = true};
        const ViewPoint view{.position_from_focus_m = {0.0, 0.0, 5.0}, .universe_to_camera = {}};
        return helios::render::build_flight_geometry(snapshot, 0, view, false).solids;
    };
    for (const double apart_m : {10.0, 6.0}) {
        const auto pieces = drawn(apart_m);
        ASSERT_EQ(pieces.size(), 2U);
        // The reel, where the line is made fast; the suit is 5 m from the camera.
        EXPECT_NEAR(norm(Vector3{pieces[0].model[12], pieces[0].model[13], pieces[0].model[14]}
                         - Vector3{apart_m, 0.0, -5.0}),
                    0.0, 1e-4);
        // The line, straight between them.
        const helios::render::SolidInstance& line = pieces[1];
        EXPECT_NEAR(norm(Vector3{line.model[0], line.model[1], line.model[2]}), apart_m, 1e-4);
        EXPECT_NEAR(
            norm(Vector3{line.model[12], line.model[13], line.model[14]} - Vector3{0.5 * apart_m, 0.0, -5.0}),
            0.0, 1e-4);
    }

    // No tether, nothing drawn.
    SceneSnapshot without;
    without.vessels.push_back(VesselView{.id = helios::sim::VesselId{0}});
    without.pilot = helios::sim::PilotView{.vessel = helios::sim::VesselId{0}, .outside = true};
    EXPECT_TRUE(helios::render::build_flight_geometry(without, 0, ViewPoint{}, false).solids.empty());
}

} // namespace
