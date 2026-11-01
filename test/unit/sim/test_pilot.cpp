#include "helios/sim/scene_snapshot.hpp"
#include "helios/sim/simulation.hpp"
#include "helios/sim/solar_system.hpp"

#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>
#include <string_view>
#include <utility>

#include "../vessel/support.hpp"

namespace {

using helios::bodies::BodyId;
using helios::core::ErrorCode;
using helios::math::norm;
using helios::math::Quaternion;
using helios::math::rotate;
using helios::math::Vector3;
using helios::sim::Pilot;
using helios::sim::Simulation;
using helios::sim::VesselId;
using helios::time::Epoch;
using helios::vessel::ControlSource;

constexpr double k_degree_rad = std::numbers::pi / 180.0;
constexpr double k_frame_s = 1.0 / 64.0;
// The habitat's room, in the vessel's axes: the floor is 1.3 m towards the tail from the
// vessel's origin, and legs hold the body's centre 1.3 m above what is under it.
constexpr double k_standing_x_m = 0.0;

// The view of someone at the habitat's controls: facing the vessel's +z, head towards the nose.
[[nodiscard]] Quaternion at_the_controls() {
    return helios::math::from_matrix(helios::math::Matrix3{{0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0}});
}

struct Scene {
    Simulation simulation;
    VesselId vessel;
    double clock_s = 0.0;

    [[nodiscard]] const Pilot& pilot() const { return simulation.pilot().value(); }
    [[nodiscard]] const helios::sim::Vessel& craft() const { return simulation.vessel(vessel).value().get(); }

    void run(double seconds) {
        const int frames = static_cast<int>(std::lround(seconds / k_frame_s));
        for (int frame = 0; frame < frames; ++frame) {
            clock_s += k_frame_s;
            const auto advanced = simulation.advance_to(Epoch::from_seconds(clock_s).value());
            ASSERT_TRUE(advanced.has_value()) << helios::core::describe(advanced.error());
        }
    }
};

[[nodiscard]] Simulation make_universe() {
    auto universe =
        helios::sim::load_stock_solar_system(HELIOS_DATA_DIR "/solar_system", std::nullopt).value();
    return Simulation::make(std::move(universe.tree), std::move(universe.catalog), Epoch{}, {}).value();
}

// Petrel: the habitat standing on the Moon.
[[nodiscard]] Scene on_the_moon(std::string_view blueprint = "Petrel") {
    Simulation simulation = make_universe();
    const BodyId moon = simulation.catalog().find("Moon").value();
    const helios::vessel::PartCatalog catalog = helios::test::load_stock_parts();
    const VesselId vessel =
        simulation
            .add_landed_vessel(std::string{blueprint}, moon, 0.67 * k_degree_rad, 23.473 * k_degree_rad,
                               helios::test::make_demo_vessel(catalog, blueprint, Epoch{}))
            .value();
    return Scene{.simulation = std::move(simulation), .vessel = vessel};
}

// Albatross: the habitat in a 500 km orbit about the Earth, flown (so in the physics bubble).
[[nodiscard]] Scene in_orbit() {
    Simulation simulation = make_universe();
    const BodyId earth = simulation.catalog().find("Earth").value();
    // Rationale: copies, not a reference into the result of body(), which GCC takes for a
    // reference to a temporary.
    const double earth_radius_m = simulation.catalog().body(earth).value().get().mean_radius_m;
    const double mu_m3_s2 = simulation.catalog().body(earth).value().get().gravitational_parameter_m3_s2;
    const double radius_m = earth_radius_m + 500e3;
    const helios::vessel::PartCatalog catalog = helios::test::load_stock_parts();
    const VesselId vessel =
        simulation
            .add_vessel("Albatross",
                        {.position_m = {radius_m, 0.0, 0.0},
                         .velocity_m_s = {0.0, std::sqrt(mu_m3_s2 / radius_m), 0.0}},
                        earth, helios::test::make_demo_vessel(catalog, "Albatross", Epoch{}))
            .value();
    EXPECT_TRUE(simulation.set_active_vessel(vessel).has_value());
    return Scene{.simulation = std::move(simulation), .vessel = vessel};
}

TEST(Pilot, TakesTheSeatOfAVesselThatHasOne) {
    Scene scene = on_the_moon();
    EXPECT_FALSE(scene.simulation.pilot().has_value());
    EXPECT_EQ(scene.simulation.leave_seat().error().code, ErrorCode::InvalidArgument);
    ASSERT_TRUE(scene.simulation.board(scene.vessel).has_value());
    EXPECT_TRUE(scene.pilot().seated);
    // The body's centre is 0.2 m below the eyes of the seat, which are at (0.2, 0, 0.5).
    EXPECT_NEAR(norm(scene.pilot().position_m - Vector3{0.0, 0.0, 0.5}), 0.0, 1e-12);
    EXPECT_NEAR(norm(rotate(scene.pilot().view, {1.0, 0.0, 0.0}) - Vector3{0.0, 0.0, 1.0}), 0.0, 1e-9);
    EXPECT_EQ(scene.simulation.take_seat().error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(scene.simulation.board(VesselId{99}).error().code, ErrorCode::OutOfRange);

    // Heron has no seat; Osprey has one, but its cabin is no bigger than the seat.
    Scene uncrewed = on_the_moon("Heron");
    EXPECT_EQ(uncrewed.simulation.board(uncrewed.vessel).error().code, ErrorCode::InvalidArgument);
    Scene cramped = on_the_moon("Osprey");
    ASSERT_TRUE(cramped.simulation.board(cramped.vessel).has_value());
    EXPECT_EQ(cramped.simulation.leave_seat().error().code, ErrorCode::InvalidArgument);
}

TEST(Pilot, StandsWalksAndJumpsOnTheMoon) {
    Scene scene = on_the_moon();
    ASSERT_TRUE(scene.simulation.board(scene.vessel).has_value());
    ASSERT_TRUE(scene.simulation.leave_seat().has_value());
    scene.simulation.set_pilot_input({.view = at_the_controls()});
    scene.run(2.0);
    // On its legs, head towards the vessel's nose, under a sixth of a g.
    EXPECT_FALSE(scene.pilot().seated);
    EXPECT_TRUE(scene.pilot().standing);
    EXPECT_NEAR(scene.pilot().up.x, 1.0, 1e-3);
    EXPECT_NEAR(norm(scene.craft().proper_acceleration_m_s2), 1.62, 0.02);
    EXPECT_NEAR(scene.pilot().position_m.x, k_standing_x_m, 0.03);
    EXPECT_LT(norm(scene.pilot().velocity_m_s), 0.01);

    // Backwards, away from the controls: the vessel's −z.
    const double start_z_m = scene.pilot().position_m.z;
    scene.simulation.set_pilot_input({.view = at_the_controls(), .move = {-1.0, 0.0, 0.0}});
    scene.run(1.0);
    EXPECT_LT(scene.pilot().position_m.z, start_z_m - 0.4);
    EXPECT_LT(scene.pilot().velocity_m_s.z, -1.0);
    EXPECT_LE(norm(scene.pilot().velocity_m_s), 1.6); // walking pace
    EXPECT_TRUE(scene.pilot().standing);
    // On to the back wall, and stop there.
    scene.run(1.5);
    scene.simulation.set_pilot_input({.view = at_the_controls()});
    scene.run(1.5);
    EXPECT_LT(norm(scene.pilot().velocity_m_s), 0.02);
    EXPECT_LT(scene.pilot().position_m.z, -0.9);
    // The back wall is 1.3 m behind the vessel's axis; the body does not go through it.
    EXPECT_GT(scene.pilot().position_m.z, -1.3 + 0.29);

    // A jump at 2.5 m/s rises about two metres on the Moon, were the ceiling not in the way.
    scene.simulation.set_pilot_input({.view = at_the_controls(), .jump = true});
    scene.run(0.5);
    EXPECT_GT(scene.pilot().position_m.x, k_standing_x_m + 0.5);
    EXPECT_FALSE(scene.pilot().standing);
    EXPECT_LT(scene.pilot().position_m.x, 1.3 - 0.29); // below the ceiling
    scene.simulation.set_pilot_input({.view = at_the_controls()});
    scene.run(4.0);
    EXPECT_TRUE(scene.pilot().standing);
    EXPECT_NEAR(scene.pilot().position_m.x, k_standing_x_m, 0.03);

    // Back to the seat: only from within reach of it.
    EXPECT_EQ(scene.simulation.take_seat().error().code, ErrorCode::InvalidArgument);
    scene.simulation.set_pilot_input({.view = at_the_controls(), .move = {1.0, 0.0, 0.0}});
    scene.run(2.0);
    ASSERT_TRUE(scene.simulation.take_seat().has_value());
    EXPECT_TRUE(scene.pilot().seated);
    EXPECT_NEAR(scene.pilot().position_m.z, 0.5, 1e-12);
}

TEST(Pilot, FloatsInOrbitAndMovesOnlyByHoldingOn) {
    Scene scene = in_orbit();
    ASSERT_TRUE(scene.simulation.board(scene.vessel).has_value());
    ASSERT_TRUE(scene.simulation.leave_seat().has_value());
    // Trying to walk in mid-air does nothing.
    scene.simulation.set_pilot_input({.view = at_the_controls(), .move = {-1.0, 1.0, 0.0}, .jump = true});
    scene.run(2.0);
    EXPECT_TRUE(scene.simulation.in_bubble(scene.vessel));
    EXPECT_FALSE(scene.pilot().standing);
    EXPECT_EQ(scene.pilot().up, Vector3{});
    EXPECT_NEAR(norm(scene.pilot().position_m - Vector3{0.0, 0.0, 0.5}), 0.0, 0.01);
    EXPECT_LT(norm(scene.pilot().velocity_m_s), 0.005);
    // The panel is in reach ahead; the far wall behind is not.
    EXPECT_TRUE(scene.pilot().in_reach);

    // Take hold of the window ahead (the eyes look over the panel) and pull in towards it.
    scene.simulation.set_pilot_input({.view = at_the_controls(), .move = {1.0, 0.0, 0.0}, .grab = true});
    scene.run(0.5);
    EXPECT_TRUE(scene.pilot().grabbing);
    EXPECT_NEAR(scene.pilot().grip_m.z, 0.5 + 0.8, 0.01); // the inside of the window
    EXPECT_GT(scene.pilot().position_m.z, 0.5 + 0.1);
    // Holding on without moving the arm: the body comes to rest where the arm has it.
    scene.simulation.set_pilot_input({.view = at_the_controls(), .grab = true});
    scene.run(1.0);
    EXPECT_TRUE(scene.pilot().grabbing);
    EXPECT_LT(norm(scene.pilot().velocity_m_s), 0.02);
    // Push away: at arm's length the hand lets go by itself, and the body flies on.
    scene.simulation.set_pilot_input({.view = at_the_controls(), .move = {-1.0, 0.0, 0.0}, .grab = true});
    scene.run(1.0);
    EXPECT_FALSE(scene.pilot().grabbing);
    const Vector3 coasting_m_s = scene.pilot().velocity_m_s;
    EXPECT_LT(coasting_m_s.z, -0.8);
    EXPECT_GT(coasting_m_s.z, -1.3);
    // Nothing slows a body in mid-air.
    scene.simulation.set_pilot_input({.view = at_the_controls()});
    scene.run(0.25);
    EXPECT_NEAR(norm(scene.pilot().velocity_m_s - coasting_m_s), 0.0, 5e-3);
    // Grabbing at nothing takes hold of nothing.
    EXPECT_FALSE(scene.pilot().in_reach);
    scene.simulation.set_pilot_input({.view = at_the_controls(), .grab = true});
    scene.run(0.1);
    EXPECT_FALSE(scene.pilot().grabbing);
    // The back wall stops the body, and there it stays, more or less.
    scene.simulation.set_pilot_input({.view = at_the_controls()});
    scene.run(3.0);
    EXPECT_GT(scene.pilot().position_m.z, -1.3 + 0.29);
}

// The pilot is a part of what moves: pushing off one way sends the vessel the other way, by
// as much momentum.
TEST(Pilot, PushingOffMovesTheVesselTheOtherWay) {
    Scene pushed = in_orbit();
    Scene still = in_orbit();
    for (Scene* scene : {&pushed, &still}) {
        ASSERT_TRUE(scene->simulation.board(scene->vessel).has_value());
        scene->simulation.set_pilot_input({.view = at_the_controls()});
        scene->run(1.0);
    }
    ASSERT_TRUE(pushed.simulation.leave_seat().has_value());
    pushed.simulation.set_pilot_input({.view = at_the_controls(), .move = {-1.0, 0.0, 0.0}, .grab = true});
    pushed.run(1.0);
    ASSERT_FALSE(pushed.pilot().grabbing); // let go at arm's length
    still.run(1.0);

    const Vector3 pilot_m_s = pushed.pilot().velocity_m_s; // in the vessel, vessel axes
    ASSERT_LT(pilot_m_s.z, -0.3);
    const Vector3 vessel_m_s = rotate(helios::math::conjugate(pushed.craft().attitude.orientation),
                                      pushed.craft().state.state_in_domain.velocity_m_s
                                          - still.craft().state.state_in_domain.velocity_m_s);
    const double vessel_mass_kg = pushed.craft().systems->mass_kg(pushed.simulation.now());
    // 80 kg leaving at pilot_m_s relative to a vessel of 7 t.
    EXPECT_NEAR(vessel_mass_kg * vessel_m_s.z, -80.0 * pilot_m_s.z, 0.15 * 80.0 * std::abs(pilot_m_s.z));
    EXPECT_GT(vessel_m_s.z, 0.0);
    // The push was off the vessel's axis, so it also turned the vessel a little.
    EXPECT_GT(
        norm(pushed.craft().attitude.angular_velocity_rad_s - still.craft().attitude.angular_velocity_rad_s),
        1e-6);
}

TEST(Pilot, ThrustPressesAFloatingPilotToTheFloor) {
    Scene scene = in_orbit();
    ASSERT_TRUE(scene.simulation.board(scene.vessel).has_value());
    ASSERT_TRUE(scene.simulation.leave_seat().has_value());
    scene.simulation.set_pilot_input({.view = at_the_controls()});
    scene.run(0.5);
    EXPECT_FALSE(scene.pilot().standing);
    for (const auto& [signal, value] : {std::pair{"engine/throttle", 1.0}, std::pair{"staging/stage", 1.0}}) {
        ASSERT_TRUE(scene.simulation.command(scene.vessel, signal, ControlSource::Pilot, value).has_value());
    }
    scene.run(3.0);
    // 30 kN on about 7 t: 0.4 g towards the tail.
    EXPECT_NEAR(norm(scene.craft().proper_acceleration_m_s2), 30'000.0 / 7'080.0, 0.2);
    EXPECT_TRUE(scene.pilot().standing);
    EXPECT_NEAR(scene.pilot().up.x, 1.0, 0.01);
    EXPECT_NEAR(scene.pilot().position_m.x, k_standing_x_m, 0.05);
    // Engine off: the floor lets go again.
    ASSERT_TRUE(
        scene.simulation.command(scene.vessel, "engine/throttle", ControlSource::Pilot, 0.0).has_value());
    ASSERT_TRUE(
        scene.simulation.command(scene.vessel, "engine/ignition", ControlSource::Pilot, 0.0).has_value());
    scene.run(1.0);
    EXPECT_FALSE(scene.pilot().standing);
}

// A cabin made for the tests: a room two metres each way around the eyes of a seat that faces
// the nose, with an 8 kg block floating half a metre ahead of the eyes.
constexpr const char* k_test_cabin = R"(
[[part]]
id = "test.cabin"
dry_mass_kg = 3000.0
shape = { kind = "cylinder", radius_m = 2.0, length_m = 3.0 }

[part.cockpit]
walkable = true
boxes = [
  { centre_m = [1.05, 0.0, 0.0], size_m = [0.1, 2.2, 2.2] },
  { centre_m = [-1.05, 0.0, 0.0], size_m = [0.1, 2.2, 2.2] },
  { centre_m = [0.0, 1.05, 0.0], size_m = [2.2, 0.1, 2.2] },
  { centre_m = [0.0, -1.05, 0.0], size_m = [2.2, 0.1, 2.2] },
  { centre_m = [0.0, 0.0, 1.05], size_m = [2.2, 2.2, 0.1] },
  { centre_m = [0.0, 0.0, -1.05], size_m = [2.2, 2.2, 0.1] },
]

[[part.cockpit.item]]
name = "block"
position_m = [0.5, 0.0, 0.0]
size_m = [0.2, 0.2, 0.2]
mass_kg = 8.0
)";

// The test cabin alone in a 500 km orbit, flown.
[[nodiscard]] Scene in_the_test_cabin() {
    Simulation simulation = make_universe();
    const BodyId earth = simulation.catalog().find("Earth").value();
    const double earth_radius_m = simulation.catalog().body(earth).value().get().mean_radius_m;
    const double mu_m3_s2 = simulation.catalog().body(earth).value().get().gravitational_parameter_m3_s2;
    const double radius_m = earth_radius_m + 500e3;
    helios::vessel::PartCatalog catalog = helios::test::load_stock_parts();
    EXPECT_TRUE(catalog.load_toml(k_test_cabin).has_value());
    helios::vessel::Assembly assembly;
    EXPECT_TRUE(assembly
                    .add_part({.name = "cabin",
                               .datasheet = catalog.part("test.cabin").value(),
                               .parent = {},
                               .position_in_parent_m = {},
                               .orientation_in_parent = {}})
                    .has_value());
    const VesselId vessel = simulation
                                .add_vessel("Test cabin",
                                            {.position_m = {radius_m, 0.0, 0.0},
                                             .velocity_m_s = {0.0, std::sqrt(mu_m3_s2 / radius_m), 0.0}},
                                            earth,
                                            helios::vessel::VesselSystems::make(
                                                std::move(assembly), catalog.interfaces(), {}, Epoch{})
                                                .value())
                                .value();
    EXPECT_TRUE(simulation.set_active_vessel(vessel).has_value());
    return Scene{.simulation = std::move(simulation), .vessel = vessel};
}

// One key does the obvious thing, and the simulation says beforehand what that is.
TEST(Pilot, IsOfferedWhatCanBeDoneWhereTheyAre) {
    using helios::sim::PilotOffer;
    Scene scene = in_the_test_cabin();
    EXPECT_EQ(scene.simulation.interact().error().code, ErrorCode::InvalidArgument); // nobody aboard
    ASSERT_TRUE(scene.simulation.board(scene.vessel).has_value());
    EXPECT_EQ(scene.pilot().offer, PilotOffer::LeaveSeat);
    ASSERT_TRUE(scene.simulation.interact().has_value());
    EXPECT_FALSE(scene.pilot().seated);
    // Still by the seat, looking at the block ahead: the block is what the look is on.
    scene.simulation.set_pilot_input({});
    scene.run(0.1);
    EXPECT_EQ(scene.pilot().offer, PilotOffer::PickUp);
    EXPECT_EQ(scene.pilot().offer_item, "block");
    EXPECT_FALSE(scene.pilot().in_reach); // an item is not a handhold
    // Looking away from it, the seat is what there is.
    const Quaternion aside = helios::math::from_axis_angle({0.0, 0.0, 1.0}, 0.5 * std::numbers::pi);
    scene.simulation.set_pilot_input({.view = aside});
    scene.run(0.1);
    EXPECT_EQ(scene.pilot().offer, PilotOffer::TakeSeat);
    ASSERT_TRUE(scene.simulation.interact().has_value());
    EXPECT_TRUE(scene.pilot().seated);
    EXPECT_EQ(scene.pilot().offer, PilotOffer::LeaveSeat);

    // A cabin no bigger than its seat offers nothing.
    Scene cramped = on_the_moon("Osprey");
    ASSERT_TRUE(cramped.simulation.board(cramped.vessel).has_value());
    EXPECT_EQ(cramped.pilot().offer, PilotOffer::None);
    EXPECT_EQ(cramped.simulation.interact().error().code, ErrorCode::InvalidArgument);
    EXPECT_TRUE(cramped.simulation.cabin_items().empty());
}

// The other way to get moving in free fall: throw something.
TEST(Pilot, PicksUpCarriesAndThrowsWithTheMomentumItTakes) {
    using helios::sim::PilotOffer;
    Scene scene = in_the_test_cabin();
    ASSERT_TRUE(scene.simulation.board(scene.vessel).has_value());
    ASSERT_EQ(scene.simulation.cabin_items().size(), 1U);
    EXPECT_EQ(scene.simulation.cabin_items()[0].name, "block");
    EXPECT_NEAR(norm(scene.simulation.cabin_items()[0].position_m - Vector3{0.5, 0.0, 0.0}), 0.0, 1e-12);
    EXPECT_EQ(scene.simulation.throw_item().error().code, ErrorCode::InvalidArgument); // nothing in hand
    ASSERT_TRUE(scene.simulation.leave_seat().has_value());
    scene.simulation.set_pilot_input({});
    scene.run(0.1);
    ASSERT_EQ(scene.pilot().offer, PilotOffer::PickUp);
    ASSERT_TRUE(scene.simulation.interact().has_value());
    EXPECT_EQ(scene.pilot().held_item, 0U);
    EXPECT_TRUE(scene.simulation.cabin_items()[0].held);
    EXPECT_EQ(scene.pilot().offer, PilotOffer::PutDown);
    EXPECT_EQ(scene.pilot().offer_item, "block");
    // In hand it goes where the pilot looks, and nothing has moved yet.
    scene.run(0.5);
    EXPECT_LT(norm(scene.pilot().velocity_m_s), 1e-3);
    EXPECT_NEAR(norm(scene.simulation.cabin_items()[0].position_m - Vector3{0.5, 0.0, 0.0}), 0.0, 5e-3);

    // 40 N s on 8 kg: they part at 5 m/s, and share no momentum between them.
    ASSERT_TRUE(scene.simulation.throw_item().has_value());
    EXPECT_FALSE(scene.pilot().held_item.has_value());
    const Vector3 pilot_m_s = scene.pilot().velocity_m_s;
    const Vector3 block_m_s = scene.simulation.cabin_items()[0].velocity_m_s;
    EXPECT_NEAR(block_m_s.x - pilot_m_s.x, 5.0, 1e-9);
    EXPECT_NEAR(norm(80.0 * pilot_m_s + 8.0 * block_m_s), 0.0, 0.1);
    EXPECT_NEAR(pilot_m_s.x, -5.0 * 8.0 / 88.0, 1e-3);
    // The pilot drifts backwards, towards the wall 0.7 m behind the ball of the body. The
    // block has by then hit the wall ahead, which jolts the vessel but is no floor to stand on.
    scene.run(0.5);
    EXPECT_NEAR(scene.pilot().velocity_m_s.x, pilot_m_s.x, 0.03);
    EXPECT_NEAR(scene.pilot().position_m.x, 0.5 * pilot_m_s.x, 0.02);
    EXPECT_FALSE(scene.pilot().standing);
    EXPECT_LT(scene.simulation.cabin_items()[0].velocity_m_s.x, 4.0);

    // Putting down is letting go without a push.
    Scene gentle = in_the_test_cabin();
    ASSERT_TRUE(gentle.simulation.board(gentle.vessel).has_value());
    ASSERT_TRUE(gentle.simulation.leave_seat().has_value());
    gentle.simulation.set_pilot_input({});
    gentle.run(0.1);
    ASSERT_TRUE(gentle.simulation.interact().has_value());
    ASSERT_TRUE(gentle.simulation.interact().has_value());
    EXPECT_FALSE(gentle.simulation.cabin_items()[0].held);
    gentle.run(1.0);
    EXPECT_LT(norm(gentle.simulation.cabin_items()[0].velocity_m_s), 0.01);
    EXPECT_LT(norm(gentle.pilot().velocity_m_s), 0.01);
    // Sitting down with something in hand leaves it where it was.
    ASSERT_TRUE(gentle.simulation.interact().has_value());
    ASSERT_TRUE(gentle.pilot().held_item.has_value());
    const Quaternion aside = helios::math::from_axis_angle({0.0, 0.0, 1.0}, 0.5 * std::numbers::pi);
    gentle.simulation.set_pilot_input({.view = aside});
    gentle.run(0.1);
    ASSERT_TRUE(gentle.simulation.take_seat().has_value());
    EXPECT_FALSE(gentle.simulation.cabin_items()[0].held);
}

// What is loose in a cabin has weight when the vessel has, whether or not anyone is up and about.
TEST(Pilot, LooseItemsLieOnTheFloorOnTheMoonAndFloatInOrbit) {
    Scene moon = on_the_moon();
    ASSERT_TRUE(moon.simulation.board(moon.vessel).has_value());
    ASSERT_EQ(moon.simulation.cabin_items().size(), 2U);
    moon.run(3.0);
    EXPECT_TRUE(moon.pilot().seated);
    const helios::sim::CabinItem crate = moon.simulation.cabin_items()[0];
    EXPECT_EQ(crate.name, "crate");
    // The floor is 1.3 m towards the tail from the vessel's origin; the crate is 0.4 m high.
    EXPECT_NEAR(crate.position_m.x, -1.3 + 0.2, 0.02);
    EXPECT_LT(norm(crate.velocity_m_s), 0.02);

    Scene orbit = in_orbit();
    ASSERT_TRUE(orbit.simulation.board(orbit.vessel).has_value());
    const Vector3 stowed_m = orbit.simulation.cabin_items()[0].position_m;
    orbit.run(2.0);
    EXPECT_NEAR(norm(orbit.simulation.cabin_items()[0].position_m - stowed_m), 0.0, 0.01);
    // With the engine lit it has weight, and the floor it was stowed on holds it.
    for (const auto& [signal, value] : {std::pair{"engine/throttle", 1.0}, std::pair{"staging/stage", 1.0}}) {
        ASSERT_TRUE(orbit.simulation.command(orbit.vessel, signal, ControlSource::Pilot, value).has_value());
    }
    orbit.run(3.0);
    EXPECT_NEAR(orbit.simulation.cabin_items()[0].position_m.x, -1.3 + 0.2, 0.03);
    EXPECT_LT(norm(orbit.simulation.cabin_items()[0].velocity_m_s), 0.05);
}

TEST(Pilot, IsInTheSnapshotRelativeToTheCentreOfMass) {
    Scene scene = on_the_moon();
    const auto focus = helios::sim::Focus::vessel(scene.vessel);
    EXPECT_FALSE(helios::sim::build_snapshot(scene.simulation, focus).value().pilot.has_value());
    ASSERT_TRUE(scene.simulation.board(scene.vessel).has_value());
    ASSERT_TRUE(scene.simulation.leave_seat().has_value());
    scene.simulation.set_pilot_input({.view = at_the_controls()});
    scene.run(2.0);
    const helios::sim::SceneSnapshot snapshot = helios::sim::build_snapshot(scene.simulation, focus).value();
    ASSERT_TRUE(snapshot.pilot.has_value());
    EXPECT_EQ(snapshot.pilot->vessel, scene.vessel);
    EXPECT_FALSE(snapshot.pilot->seated);
    EXPECT_TRUE(snapshot.pilot->standing);
    EXPECT_NEAR(snapshot.pilot->up.x, 1.0, 1e-3);
    const Vector3 centre_of_mass_m =
        scene.craft().systems->mass_properties(scene.simulation.now()).value().centre_of_mass_m;
    EXPECT_LT(centre_of_mass_m.x, -1.0); // in the tank below the habitat
    EXPECT_NEAR(norm(snapshot.pilot->position_m - (scene.pilot().position_m - centre_of_mass_m)), 0.0, 1e-12);
    // What the interact key would do, and the loose items, come with it.
    EXPECT_EQ(snapshot.pilot->offer, helios::sim::PilotOffer::TakeSeat);
    EXPECT_FALSE(snapshot.pilot->holding);
    ASSERT_EQ(snapshot.items.size(), 2U);
    EXPECT_EQ(snapshot.items[0].name, "crate");
    EXPECT_FALSE(snapshot.items[0].held);
    EXPECT_NEAR(norm(snapshot.items[0].position_m
                     - (scene.simulation.cabin_items()[0].position_m - centre_of_mass_m)),
                0.0, 1e-12);
    // The parts are given the same way, so the pilot is inside the habitat's part.
    EXPECT_NEAR(norm(snapshot.vessels[scene.vessel.index].parts.front().position_m + centre_of_mass_m), 0.0,
                1e-12);
}

TEST(Pilot, StaysPutRelativeToTheVesselAboveTheWarpOfThePhysics) {
    Scene scene = in_orbit();
    ASSERT_TRUE(scene.simulation.board(scene.vessel).has_value());
    ASSERT_TRUE(scene.simulation.leave_seat().has_value());
    scene.simulation.set_pilot_input({.view = at_the_controls(), .move = {-1.0, 0.0, 0.0}, .grab = true});
    scene.run(0.5);
    const Vector3 before_m = scene.pilot().position_m;
    scene.simulation.time_warp().request_level(helios::sim::k_real_time_level + 3); // 1000×
    for (int frame = 0; frame < 30; ++frame) {
        ASSERT_TRUE(scene.simulation.advance(1.0 / 60.0).has_value());
    }
    EXPECT_FALSE(scene.simulation.in_bubble(scene.vessel));
    EXPECT_EQ(scene.pilot().position_m, before_m);
    EXPECT_FALSE(scene.pilot().grabbing);
}

// --- Outside (BRIEFING D31) ---------------------------------------------------------------------

void provide_suit(Simulation& simulation) {
    const helios::vessel::PartCatalog catalog = helios::test::load_stock_parts();
    simulation.provide_suit(helios::test::make_demo_vessel(catalog, "EVA suit", Epoch{}));
}

// Out of the seat and through the airlock, which is within reach of the habitat's seat.
void go_outside(Scene& scene) {
    provide_suit(scene.simulation);
    ASSERT_TRUE(scene.simulation.board(scene.vessel).has_value());
    ASSERT_TRUE(scene.simulation.leave_seat().has_value());
    const auto started = scene.simulation.go_outside();
    ASSERT_TRUE(started.has_value()) << helios::core::describe(started.error());
    scene.run(20.5);
    ASSERT_TRUE(scene.pilot().outside);
}

[[nodiscard]] double suit_reading(const Scene& scene, std::string_view signal) {
    const helios::vessel::ControlBus& bus =
        scene.simulation.vessel(scene.pilot().vessel).value().get().systems->bus();
    return bus.value(bus.find(signal).value());
}

TEST(Eva, ThePilotGoesOutThroughTheAirlockAsAVesselOfItsOwn) {
    Scene scene = in_orbit();
    Simulation& simulation = scene.simulation;
    ASSERT_TRUE(simulation.board(scene.vessel).has_value());
    EXPECT_EQ(simulation.go_outside().error().code, ErrorCode::InvalidArgument) << "from the seat";
    ASSERT_TRUE(simulation.leave_seat().has_value());
    EXPECT_EQ(simulation.go_outside().error().code, ErrorCode::InvalidArgument) << "without a suit";
    provide_suit(simulation);
    ASSERT_TRUE(simulation.go_outside().has_value());

    // Twenty seconds to put the suit on and let the air out, with nothing else to do.
    EXPECT_DOUBLE_EQ(scene.pilot().airlock_cycle_s, 20.0);
    EXPECT_EQ(scene.pilot().offer, helios::sim::PilotOffer::None);
    EXPECT_EQ(simulation.take_seat().error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(simulation.go_outside().error().code, ErrorCode::InvalidArgument);
    scene.run(10.0);
    EXPECT_FALSE(scene.pilot().outside);
    EXPECT_NEAR(scene.pilot().airlock_left_s, 10.0, 0.05);
    EXPECT_EQ(simulation.vessels().size(), 1U);

    scene.run(10.5);
    ASSERT_EQ(simulation.vessels().size(), 2U);
    const VesselId suit{1};
    EXPECT_TRUE(scene.pilot().outside);
    EXPECT_TRUE(scene.pilot().seated) << "the suit is worn, not walked about in";
    EXPECT_FALSE(helios::sim::build_snapshot(simulation, helios::sim::Focus::vessel(suit))
                     .value()
                     .pilot->ground.has_value());
    EXPECT_EQ(scene.pilot().vessel, suit);
    EXPECT_EQ(scene.pilot().airlock_left_s, 0.0);
    // The pilot's body is what is flown now, and the habitat is a body beside it.
    EXPECT_EQ(simulation.active_vessel(), suit);
    EXPECT_TRUE(simulation.in_bubble(suit));
    EXPECT_TRUE(simulation.in_bubble(scene.vessel));
    const helios::sim::Vessel& worn = simulation.vessels()[suit.index];
    EXPECT_EQ(worn.status, helios::sim::VesselStatus::Flying);
    // Just clear of the hull (2 m in radius) and moving with it.
    const Vector3 apart_m =
        worn.state.state_in_domain.position_m - scene.craft().state.state_in_domain.position_m;
    EXPECT_GT(norm(apart_m), 2.4);
    EXPECT_LT(norm(apart_m), 5.0);
    EXPECT_LT(
        norm(worn.state.state_in_domain.velocity_m_s - scene.craft().state.state_in_domain.velocity_m_s),
        0.05);
    // Facing the habitat, as if having stepped back out of the hatch.
    EXPECT_LT(helios::math::dot(rotate(worn.attitude.orientation, {0.0, 0.0, 1.0}), apart_m / norm(apart_m)),
              -0.5);
    // Still at the airlock: the one key would take the pilot back in.
    EXPECT_EQ(scene.pilot().offer, helios::sim::PilotOffer::ComeInside);
}

TEST(Eva, TheThrusterPackFliesTheSuitAndItComesBackInAsItWas) {
    Scene scene = in_orbit();
    Simulation& simulation = scene.simulation;
    go_outside(scene);
    const VesselId suit = scene.pilot().vessel;
    const auto relative_m_s = [&] {
        return simulation.vessel(suit).value().get().state.state_in_domain.velocity_m_s
               - scene.craft().state.state_in_domain.velocity_m_s;
    };
    const Vector3 before_m_s = relative_m_s();
    EXPECT_DOUBLE_EQ(suit_reading(scene, "pack/nitrogen_fraction"), 1.0);

    // 30 N on 182 kg for two seconds, backwards: away from the hull the pilot faces.
    ASSERT_TRUE(simulation.command(suit, "translation/z", ControlSource::Pilot, -1.0).has_value());
    scene.run(2.0);
    ASSERT_TRUE(simulation.command(suit, "translation/z", ControlSource::Pilot, 0.0).has_value());
    EXPECT_NEAR(norm(relative_m_s() - before_m_s), 30.0 / 182.0 * 2.0, 0.03);
    EXPECT_NEAR(suit_reading(scene, "pack/nitrogen_fraction"), 1.0 - 0.1 / 12.0, 1e-4);
    const double fraction = suit_reading(scene, "pack/nitrogen_fraction");
    scene.run(0.1);
    EXPECT_EQ(scene.pilot().offer, helios::sim::PilotOffer::ComeInside) << "still within reach";

    // Back in: the suit is put away at once, and the cabin is the pilot's again after the wait.
    const auto back = simulation.come_inside();
    ASSERT_TRUE(back.has_value()) << helios::core::describe(back.error());
    EXPECT_EQ(simulation.vessel(suit).value().get().status, helios::sim::VesselStatus::Stowed);
    EXPECT_FALSE(simulation.in_bubble(suit));
    EXPECT_EQ(simulation.active_vessel(), scene.vessel);
    EXPECT_EQ(scene.pilot().vessel, scene.vessel);
    EXPECT_FALSE(scene.pilot().outside);
    EXPECT_FALSE(scene.pilot().seated);
    EXPECT_DOUBLE_EQ(scene.pilot().airlock_left_s, 20.0);
    EXPECT_EQ(simulation.command(suit, "translation/z", ControlSource::Pilot, 1.0).error().code,
              ErrorCode::InvalidArgument);
    scene.run(20.5);
    EXPECT_EQ(scene.pilot().airlock_left_s, 0.0);
    EXPECT_NE(scene.pilot().offer, helios::sim::PilotOffer::None);
    EXPECT_TRUE(simulation.in_bubble(scene.vessel));

    // Out again: the same suit, with the nitrogen it came in with.
    ASSERT_TRUE(simulation.go_outside().has_value());
    scene.run(20.5);
    ASSERT_TRUE(scene.pilot().outside);
    EXPECT_EQ(simulation.vessels().size(), 2U);
    EXPECT_EQ(scene.pilot().vessel, suit);
    EXPECT_NEAR(suit_reading(scene, "pack/nitrogen_fraction"), fraction, 1e-9);

    // Away from the airlock there is no way in.
    ASSERT_TRUE(simulation.command(suit, "translation/z", ControlSource::Pilot, -1.0).has_value());
    scene.run(8.0);
    EXPECT_EQ(scene.pilot().offer, helios::sim::PilotOffer::None);
    EXPECT_EQ(simulation.come_inside().error().code, ErrorCode::InvalidArgument);
}

// Out of a lander standing on the Moon: the suit falls to the ground beside it and stays there,
// and the lander does not move.
TEST(Eva, OutOfALanderOnTheMoonThePilotComesDownBesideIt) {
    Scene scene = on_the_moon();
    Simulation& simulation = scene.simulation;
    ASSERT_TRUE(simulation.set_active_vessel(scene.vessel).has_value());
    const helios::sim::LandedPlace place = *scene.craft().landed;
    go_outside(scene);
    const VesselId suit = scene.pilot().vessel;
    scene.run(15.0);
    const helios::sim::Vessel& worn = simulation.vessels()[suit.index];
    EXPECT_EQ(worn.status, helios::sim::VesselStatus::Landed);
    EXPECT_LT(norm(worn.state.state_in_domain.position_m - scene.craft().state.state_in_domain.position_m),
              12.0);
    EXPECT_LT(norm(worn.state.state_in_domain.position_m),
              norm(scene.craft().state.state_in_domain.position_m));
    EXPECT_EQ(scene.craft().status, helios::sim::VesselStatus::Landed);
    EXPECT_EQ(scene.craft().landed->position_m, place.position_m);
}

// The suit breathes while it is worn, and what is left of its oxygen stays in it while it is
// put away in the cabin.
TEST(Eva, TheSuitUsesOxygenWhileItIsWornAndKeepsTheRest) {
    Scene scene = in_orbit();
    Simulation& simulation = scene.simulation;
    go_outside(scene);
    const VesselId suit = scene.pilot().vessel;
    const auto oxygen = [&]() {
        const helios::vessel::ControlBus& bus = simulation.vessel(suit).value().get().systems->bus();
        return bus.value(bus.find("suit/oxygen_fraction").value());
    };
    EXPECT_GT(oxygen(), 0.9999) << "a fresh suit's is full";

    // A minute of breathing: 6e-4 kg of the 0.36 kg the suit carries (1e-5 kg/s).
    const double before = oxygen();
    scene.run(60.0);
    const double after = oxygen();
    EXPECT_NEAR(after - before, -60.0 * 1.0e-5 / 0.36, 1e-6);
    const helios::sim::SceneSnapshot worn =
        helios::sim::build_snapshot(simulation, helios::sim::Focus::vessel(suit)).value();
    ASSERT_TRUE(worn.pilot.has_value());
    EXPECT_NEAR(worn.pilot->oxygen, after, 1e-9) << "the picture is told of it";

    // In the cabin the suit is stowed, and stowed it breathes no more.
    const auto back = simulation.come_inside();
    ASSERT_TRUE(back.has_value()) << helios::core::describe(back.error());
    scene.run(120.0);
    ASSERT_TRUE(simulation.vessel(suit).has_value());
    EXPECT_EQ(simulation.vessel(suit).value().get().status, helios::sim::VesselStatus::Stowed);
    EXPECT_DOUBLE_EQ(oxygen(), after);
    const helios::sim::SceneSnapshot inside =
        helios::sim::build_snapshot(simulation, helios::sim::Focus::vessel(scene.vessel)).value();
    ASSERT_TRUE(inside.pilot.has_value());
    EXPECT_LT(inside.pilot->oxygen, 0.0) << "the cabin has no gauge to show";

    // Out again: it breathes on from where it was put away. Of the two and a half minutes in
    // the locker and in the airlock, only the half second it has been in the world is breathed.
    ASSERT_TRUE(simulation.go_outside().has_value());
    scene.run(20.5);
    ASSERT_TRUE(scene.pilot().outside);
    EXPECT_NEAR(after - oxygen(), 0.5 * 1.0e-5 / 0.36, 1e-6);
}

// --- A suit that settles its own spin (BRIEFING D34) -----------------------------------------------

TEST(Eva, ASuitLeftTurningSettlesToASlowTurn) {
    Scene scene = in_orbit();
    Simulation& simulation = scene.simulation;
    go_outside(scene);
    const VesselId suit = scene.pilot().vessel;
    scene.run(2.0);
    const auto spin_rad_s = [&] {
        return norm(simulation.vessels()[suit.index].attitude.angular_velocity_rad_s);
    };
    EXPECT_LT(spin_rad_s(), 1e-3) << "nothing is turning yet";

    // The pack rolls the suit up to speed, and then lets go.
    ASSERT_TRUE(simulation.command(suit, "attitude/roll", ControlSource::Pilot, 1.0).has_value());
    scene.run(4.0);
    ASSERT_TRUE(simulation.command(suit, "attitude/roll", ControlSource::Pilot, 0.0).has_value());
    const double turning = spin_rad_s();
    EXPECT_GT(turning, 1.0);

    // Out here nothing else acts on the suit: the turn settles at 0.3 per second.
    scene.run(6.0);
    const double expected = turning * std::exp(-0.3 * 6.0);
    EXPECT_LT(spin_rad_s(), 1.5 * expected);
    EXPECT_GT(spin_rad_s(), 0.5 * expected);
}

// --- On foot outside (BRIEFING D32) -------------------------------------------------------------

// How the suit stands: the height of its centre of mass over the mean surface, and how far its
// head end is from straight up.
struct Stance {
    double height_m = 0.0;
    double lean_rad = 0.0;
};

[[nodiscard]] Stance stance_of(const Scene& scene) {
    const helios::sim::Vessel& suit = scene.simulation.vessels()[scene.pilot().vessel.index];
    const BodyId moon = suit.state.domain;
    const double radius_m = scene.simulation.catalog().body(moon).value().get().mean_radius_m;
    const Vector3& position_m = suit.state.state_in_domain.position_m;
    const Vector3 up = position_m / norm(position_m);
    const Vector3 head = helios::math::rotate(suit.attitude.orientation, Vector3{1.0, 0.0, 0.0});
    return {.height_m = norm(position_m) - radius_m,
            .lean_rad = std::acos(std::clamp(helios::math::dot(head, up), -1.0, 1.0))};
}

// Where the suit faces, in the axes of the ground: its +z, as the wearer's eyes are set in it.
[[nodiscard]] Vector3 facing_of(const Scene& scene) {
    const helios::sim::Vessel& suit = scene.simulation.vessels()[scene.pilot().vessel.index];
    return helios::math::rotate(suit.landed.value().orientation, Vector3{0.0, 0.0, 1.0});
}

TEST(Eva, OnTheMoonThePilotStandsWalksTurnsAndJumpsInTheSuit) {
    Scene scene = on_the_moon();
    Simulation& simulation = scene.simulation;
    ASSERT_TRUE(simulation.set_active_vessel(scene.vessel).has_value());
    go_outside(scene);
    const VesselId suit = scene.pilot().vessel;
    const auto worn = [&]() -> const helios::sim::Vessel& { return simulation.vessels()[suit.index]; };
    scene.run(15.0);

    // Down from the hatch and on its feet: standing straight, the suit held just clear of the
    // ground by the legs, and pinned there like any landed vessel once it is still.
    EXPECT_FALSE(scene.pilot().seated);
    EXPECT_TRUE(scene.pilot().standing);
    EXPECT_NEAR(norm(scene.pilot().up), 1.0, 1e-9);
    ASSERT_EQ(worn().status, helios::sim::VesselStatus::Landed);
    EXPECT_FALSE(simulation.in_bubble(suit)) << "nothing flies: nothing is simulated";
    const Stance standing = stance_of(scene);
    EXPECT_LT(standing.lean_rad, 0.06);
    EXPECT_GT(standing.height_m, 0.9);
    EXPECT_LT(standing.height_m, 1.2);
    const helios::sim::LandedPlace stood = *worn().landed;
    // The look and the weight are in the ground's axes, which the picture is told of.
    const helios::sim::SceneSnapshot snapshot =
        helios::sim::build_snapshot(simulation, helios::sim::Focus::vessel(suit)).value();
    ASSERT_TRUE(snapshot.pilot.has_value());
    ASSERT_TRUE(snapshot.pilot->ground.has_value());
    const Vector3 universe_up =
        worn().state.state_in_domain.position_m / norm(worn().state.state_in_domain.position_m);
    EXPECT_LT(norm(helios::math::rotate(*snapshot.pilot->ground, snapshot.pilot->up) - universe_up), 1e-3);
    scene.run(5.0);
    EXPECT_EQ(worn().landed->position_m, stood.position_m)
        << "standing still costs nothing and moves nothing";

    // Walking: backwards, away from the lander the suit faces, at walking pace, still upright.
    const Quaternion ahead = scene.pilot().view;
    const Vector3 forward = helios::math::rotate(ahead, Vector3{1.0, 0.0, 0.0});
    EXPECT_GT(helios::math::dot(forward, facing_of(scene)), 0.99) << "the look starts where the suit faces";
    simulation.set_pilot_input({.view = ahead, .move = {-1.0, 0.0, 0.0}});
    scene.run(4.0);
    EXPECT_EQ(worn().status, helios::sim::VesselStatus::Flying) << "a body of the bubble while it moves";
    EXPECT_TRUE(scene.pilot().standing);
    EXPECT_LT(stance_of(scene).lean_rad, 0.1);
    simulation.set_pilot_input({.view = ahead});
    scene.run(3.0);
    ASSERT_EQ(worn().status, helios::sim::VesselStatus::Landed);
    const Vector3 walked_m = worn().landed->position_m - stood.position_m;
    EXPECT_NEAR(helios::math::dot(walked_m, forward), -6.0, 1.0);
    EXPECT_LT(norm(walked_m - helios::math::dot(walked_m, forward) * forward), 0.3);
    EXPECT_NEAR(stance_of(scene).height_m, standing.height_m, 0.02);

    // Looking a little aside turns the head only; looking over the shoulder turns the body.
    const Vector3 faced = facing_of(scene);
    const Vector3 up = worn().landed->position_m / norm(worn().landed->position_m);
    const auto looking = [&](double left_rad) {
        return ahead * helios::math::from_axis_angle(Vector3{0.0, 0.0, 1.0}, left_rad);
    };
    simulation.set_pilot_input({.view = looking(0.3)});
    scene.run(2.0);
    EXPECT_EQ(worn().status, helios::sim::VesselStatus::Landed);
    EXPECT_EQ(facing_of(scene), faced);
    simulation.set_pilot_input({.view = looking(0.5 * std::numbers::pi)});
    scene.run(4.0);
    ASSERT_EQ(worn().status, helios::sim::VesselStatus::Landed);
    const Vector3 turned = facing_of(scene);
    EXPECT_NEAR(helios::math::dot(turned, helios::math::cross(up, faced)), 1.0, 0.01) << "to the left";
    EXPECT_LT(stance_of(scene).lean_rad, 0.06);

    // A jump: off the ground, and back on the feet.
    const double before_m = stance_of(scene).height_m;
    simulation.set_pilot_input({.view = looking(0.5 * std::numbers::pi), .jump = true});
    double highest_m = before_m;
    bool left_the_ground = false;
    for (int step = 0; step < 40; ++step) {
        scene.run(0.1);
        highest_m = std::max(highest_m, stance_of(scene).height_m);
        left_the_ground = left_the_ground || !scene.pilot().standing;
    }
    EXPECT_TRUE(left_the_ground);
    // 2.5 m/s of leg for a body of 80 kg is 1.7 m/s with the suit on: 0.9 m up on the Moon.
    EXPECT_NEAR(highest_m - before_m, 0.9, 0.25);
    simulation.set_pilot_input({.view = looking(0.5 * std::numbers::pi)});
    scene.run(3.0);
    EXPECT_EQ(worn().status, helios::sim::VesselStatus::Landed) << "one jump for one press";
    EXPECT_NEAR(stance_of(scene).height_m, before_m, 0.02);
    EXPECT_EQ(scene.craft().status, helios::sim::VesselStatus::Landed);
}

// --- A ladder up to the hatch (BRIEFING D35) ------------------------------------------------------

// On the Moon the hatch is 4.65 m above where the suit stands, out of reach; the ladder below
// it goes from there up to the hatch.
TEST(Eva, OnTheMoonThePilotClimbsTheLadderUpToTheHatchAndDownAgain) {
    Scene scene = on_the_moon();
    Simulation& simulation = scene.simulation;
    ASSERT_TRUE(simulation.set_active_vessel(scene.vessel).has_value());
    const helios::sim::LandedPlace place = *scene.craft().landed;
    go_outside(scene);
    const VesselId suit = scene.pilot().vessel;
    const auto worn = [&]() -> const helios::sim::Vessel& { return simulation.vessels()[suit.index]; };
    scene.run(15.0);
    ASSERT_EQ(worn().status, helios::sim::VesselStatus::Landed);
    const double standing_m = stance_of(scene).height_m;
    // Down by the hull the ladder is in reach, the hatch is not.
    EXPECT_EQ(simulation.come_inside().error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(scene.pilot().offer, helios::sim::PilotOffer::TakeLadder);
    EXPECT_EQ(simulation.let_go_ladder().error().code, ErrorCode::InvalidArgument) << "nothing held yet";
    ASSERT_TRUE(simulation.interact().has_value());
    EXPECT_TRUE(simulation.on_ladder());
    EXPECT_EQ(scene.pilot().offer, helios::sim::PilotOffer::LetGo);
    EXPECT_EQ(simulation.grab_ladder().error().code, ErrorCode::InvalidArgument) << "already held";

    // Up: 4.65 m at 0.6 m/s, the arm carrying the suit's weight; at the top the hatch is in reach.
    const Quaternion look = scene.pilot().view;
    simulation.set_pilot_input({.view = look, .move = {1.0, 0.0, 0.0}});
    scene.run(9.0);
    EXPECT_EQ(worn().status, helios::sim::VesselStatus::Flying);
    EXPECT_FALSE(scene.pilot().standing);
    EXPECT_FALSE(scene.pilot().seated);
    EXPECT_NEAR(stance_of(scene).height_m - standing_m, 4.65, 0.15);
    EXPECT_LT(stance_of(scene).lean_rad, 0.1) << "head up the ladder";
    EXPECT_EQ(scene.pilot().offer, helios::sim::PilotOffer::ComeInside);
    const helios::sim::SceneSnapshot snapshot =
        helios::sim::build_snapshot(simulation, helios::sim::Focus::vessel(suit)).value();
    ASSERT_TRUE(snapshot.pilot.has_value());
    EXPECT_TRUE(snapshot.pilot->on_ladder);

    // Held there without climbing, it stays where it is.
    simulation.set_pilot_input({.view = look});
    scene.run(1.0);
    const double held_m = stance_of(scene).height_m;
    scene.run(3.0);
    EXPECT_NEAR(stance_of(scene).height_m, held_m, 0.02);

    // Down again, let go, and on the feet where the climb began. The lander has not moved.
    simulation.set_pilot_input({.view = look, .move = {-1.0, 0.0, 0.0}});
    scene.run(9.0);
    EXPECT_NEAR(stance_of(scene).height_m, standing_m, 0.15);
    EXPECT_EQ(scene.pilot().offer, helios::sim::PilotOffer::LetGo);
    ASSERT_TRUE(simulation.interact().has_value());
    EXPECT_FALSE(simulation.on_ladder());
    simulation.set_pilot_input({.view = scene.pilot().view});
    scene.run(4.0);
    EXPECT_TRUE(scene.pilot().standing);
    EXPECT_EQ(worn().status, helios::sim::VesselStatus::Landed);
    EXPECT_NEAR(stance_of(scene).height_m, standing_m, 0.05);
    EXPECT_EQ(scene.craft().status, helios::sim::VesselStatus::Landed);
    EXPECT_EQ(scene.craft().landed->position_m, place.position_m);
}

// In orbit the same ladder is a handhold: climbing it pulls the habitat the other way, and the
// two keep their momentum between them. Gravity changes both a great deal over seconds, so the
// climb is set against the same seconds spent holding still.
struct Pair {
    Vector3 suit_m;
    Vector3 suit_m_s;
    Vector3 craft_m_s;
    double suit_kg = 0.0;
    double craft_kg = 0.0;
};

[[nodiscard]] Pair hold_the_ladder_in_orbit(double climb) {
    Scene scene = in_orbit();
    Simulation& simulation = scene.simulation;
    go_outside(scene);
    const VesselId suit = scene.pilot().vessel;
    // Just out of the airlock the suit is within reach of the top of the ladder too.
    EXPECT_EQ(scene.pilot().offer, helios::sim::PilotOffer::ComeInside) << "the hatch comes first";
    const auto held = simulation.grab_ladder();
    EXPECT_TRUE(held.has_value());
    scene.run(2.0);
    // Three seconds along the ladder at 0.6 m/s (or none), and three to come to rest on it.
    simulation.set_pilot_input({.view = scene.pilot().view, .move = {climb, 0.0, 0.0}});
    scene.run(3.0);
    simulation.set_pilot_input({.view = scene.pilot().view});
    scene.run(3.0);
    EXPECT_TRUE(simulation.on_ladder());
    const helios::sim::Vessel& worn = simulation.vessels()[suit.index];
    const auto kg = [&](const helios::sim::Vessel& vessel) {
        return vessel.systems->mass_properties(simulation.now()).value().mass_kg;
    };
    return Pair{.suit_m =
                    worn.state.state_in_domain.position_m - scene.craft().state.state_in_domain.position_m,
                .suit_m_s = worn.state.state_in_domain.velocity_m_s,
                .craft_m_s = scene.craft().state.state_in_domain.velocity_m_s,
                .suit_kg = kg(worn),
                .craft_kg = kg(scene.craft())};
}

TEST(Eva, InOrbitClimbingTheLadderPullsTheHabitatTheOtherWay) {
    const Pair still = hold_the_ladder_in_orbit(0.0);
    const Pair climbed = hold_the_ladder_in_orbit(-1.0);
    // Down the ladder, 1.8 m further along the hull than holding still.
    EXPECT_NEAR(norm(climbed.suit_m - still.suit_m), 1.8, 0.2);
    // Held: moving with the hull at the end.
    EXPECT_LT(norm(climbed.suit_m_s - climbed.craft_m_s), 0.02);
    // The habitat moved, a little, and the two together gained nothing: what the arm did to the
    // one it did to the other.
    const Vector3 craft_gained_m_s = climbed.craft_m_s - still.craft_m_s;
    EXPECT_GT(norm(craft_gained_m_s), 1e-5);
    const Vector3 gained_n_s =
        climbed.suit_kg * (climbed.suit_m_s - still.suit_m_s) + climbed.craft_kg * craft_gained_m_s;
    EXPECT_LT(norm(gained_n_s), 0.5);
}

// Taking hold of the ladder in a way the arm has to turn the suit right round: it turns it at a
// steady pace and stops there, whatever the hands do meanwhile. (The arm once took a body held
// head down for one already turned, and the first push rolled it over faster and faster.)
TEST(Eva, OnTheLadderTheArmTurnsTheSuitRoundSteadily) {
    Scene scene = in_orbit();
    Simulation& simulation = scene.simulation;
    go_outside(scene);
    const VesselId suit = scene.pilot().vessel;
    const auto spin_rad_s = [&] {
        return norm(simulation.vessels()[suit.index].attitude.angular_velocity_rad_s
                    - scene.craft().attitude.angular_velocity_rad_s);
    };
    ASSERT_TRUE(simulation.grab_ladder().has_value());
    double fastest_rad_s = 0.0;
    for (int tenth = 0; tenth < 100; ++tenth) {
        // Across the ladder for three seconds, as with the A key, then still.
        simulation.set_pilot_input({.view = scene.pilot().view, .move = {0.0, tenth < 30 ? 1.0 : 0.0, 0.0}});
        scene.run(0.1);
        fastest_rad_s = std::max(fastest_rad_s, spin_rad_s());
    }
    EXPECT_TRUE(simulation.on_ladder());
    EXPECT_LT(fastest_rad_s, 1.2) << "no faster than the arm's steady turn";
    EXPECT_LT(spin_rad_s(), 0.01) << "and at rest on the ladder at the end";
}

} // namespace
