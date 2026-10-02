#include "helios/sim/scene_snapshot.hpp"
#include "helios/sim/simulation.hpp"
#include "helios/sim/solar_system.hpp"

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

} // namespace
