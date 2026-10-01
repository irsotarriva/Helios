#include "helios/sim/simulation.hpp"
#include "helios/sim/solar_system.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <numbers>
#include <optional>
#include <string_view>
#include <utility>

#include "../support/universes.hpp"
#include "../vessel/support.hpp"

// The physics bubble (BRIEFING D25): the active vessel as a rigid body that turns, touches the
// ground and lands, while its orbit stays with the propagator.
namespace {

using helios::bodies::BodyId;
using helios::core::ErrorCode;
using helios::math::cross;
using helios::math::dot;
using helios::math::norm;
using helios::math::rotate;
using helios::math::Vector3;
using helios::orbital::StateVector;
using helios::sim::Simulation;
using helios::sim::Vessel;
using helios::sim::VesselId;
using helios::sim::VesselStatus;
using helios::time::Epoch;
using helios::vessel::ControlSource;

constexpr double k_kestrel_mass_kg = 25'490.0;
constexpr double k_heron_mass_kg = 250.0 + 4'400.0 + 120.0 + 80.0;
constexpr double k_degree_rad = std::numbers::pi / 180.0;

// A 630 km circular orbit.
const StateVector k_low_orbit{.position_m = {7.0e6, 0.0, 0.0}, .velocity_m_s = {0.0, 7'546.0, 300.0}};

[[nodiscard]] Epoch at(double seconds) {
    return Epoch::from_seconds(seconds).value();
}

void command(Simulation& simulation, VesselId vessel, std::string_view signal, double value) {
    const auto commanded = simulation.command(vessel, signal, ControlSource::Pilot, value);
    ASSERT_TRUE(commanded.has_value()) << helios::core::describe(commanded.error());
}

[[nodiscard]] double reading(const Simulation& simulation, VesselId vessel, std::string_view signal) {
    const helios::vessel::ControlBus& bus = simulation.vessel(vessel).value().get().systems->bus();
    return bus.value(bus.find(signal).value());
}

// Advances in frames of 1/32 s, as a game loop would.
void run(Simulation& simulation, double from_s, double to_s) {
    constexpr double k_frame_s = 1.0 / 32.0;
    const int frames = static_cast<int>(std::lround((to_s - from_s) / k_frame_s));
    for (int frame = 1; frame <= frames; ++frame) {
        const auto advanced = simulation.advance_to(at(from_s + frame * k_frame_s));
        ASSERT_TRUE(advanced.has_value()) << helios::core::describe(advanced.error());
    }
}

struct Orbiter {
    Simulation simulation;
    VesselId kestrel;
    BodyId earth;

    [[nodiscard]] const Vessel& vessel() const { return simulation.vessel(kestrel).value().get(); }
    [[nodiscard]] Vector3 nose() const { return rotate(vessel().attitude.orientation, {1.0, 0.0, 0.0}); }
};

[[nodiscard]] Orbiter make_orbiter() {
    helios::test::EarthMoonCr3bp system = helios::test::make_earth_moon_cr3bp();
    const BodyId earth = system.earth;
    Simulation simulation =
        Simulation::make(std::move(system.tree), std::move(system.catalog), Epoch{}, {}).value();
    const helios::vessel::PartCatalog catalog = helios::test::load_stock_parts();
    const VesselId kestrel = simulation
                                 .add_vessel("Kestrel", k_low_orbit, earth,
                                             helios::test::make_demo_vessel(catalog, "Kestrel", Epoch{}))
                                 .value();
    return Orbiter{.simulation = std::move(simulation), .kestrel = kestrel, .earth = earth};
}

struct Lander {
    Simulation simulation;
    VesselId heron;
    BodyId moon;

    [[nodiscard]] const Vessel& vessel() const { return simulation.vessel(heron).value().get(); }
    [[nodiscard]] double altitude_m() const {
        return norm(vessel().state.state_in_domain.position_m)
               - simulation.catalog().body(moon).value().get().mean_radius_m;
    }
    // How upright the vessel is: 1 when its nose points straight up.
    [[nodiscard]] double uprightness() const {
        const Vector3& position_m = vessel().state.state_in_domain.position_m;
        return dot(rotate(vessel().attitude.orientation, {1.0, 0.0, 0.0}), position_m / norm(position_m));
    }
};

// The stock Solar System (the Moon turns), with Heron standing in Mare Tranquillitatis.
[[nodiscard]] Lander make_lander() {
    auto universe =
        helios::sim::load_stock_solar_system(HELIOS_DATA_DIR "/solar_system", std::nullopt).value();
    Simulation simulation =
        Simulation::make(std::move(universe.tree), std::move(universe.catalog), Epoch{}, {}).value();
    const BodyId moon = simulation.catalog().find("Moon").value();
    const helios::vessel::PartCatalog catalog = helios::test::load_stock_parts();
    const VesselId heron = simulation
                               .add_landed_vessel("Heron", moon, 0.674 * k_degree_rad, 23.473 * k_degree_rad,
                                                  helios::test::make_demo_vessel(catalog, "Heron", Epoch{}))
                               .value();
    return Lander{.simulation = std::move(simulation), .heron = heron, .moon = moon};
}

// What a cockpit's flight instruments read comes off the bus like everything else.
TEST(Navigation, TheBusReportsWhereTheVesselIsAndHowItMoves) {
    Lander lander = make_lander();
    const auto on_the_ground = [&](std::string_view signal) {
        return reading(lander.simulation, lander.heron, signal);
    };
    // Standing on the Moon: no speed over the ground, but the ground itself goes round.
    EXPECT_DOUBLE_EQ(on_the_ground("nav/altitude_m"), lander.altitude_m());
    EXPECT_GT(on_the_ground("nav/altitude_m"), 1.0);
    EXPECT_NEAR(on_the_ground("nav/surface_speed_m_s"), 0.0, 1e-9);
    EXPECT_NEAR(on_the_ground("nav/vertical_speed_m_s"), 0.0, 1e-9);
    EXPECT_NEAR(on_the_ground("nav/speed_m_s"), 4.6, 0.1);

    Orbiter orbiter = make_orbiter();
    run(orbiter.simulation, 0.0, 10.0);
    const StateVector& state = orbiter.vessel().state.state_in_domain;
    const double earth_radius_m =
        orbiter.simulation.catalog().body(orbiter.earth).value().get().mean_radius_m;
    EXPECT_DOUBLE_EQ(reading(orbiter.simulation, orbiter.kestrel, "nav/altitude_m"),
                     norm(state.position_m) - earth_radius_m);
    EXPECT_DOUBLE_EQ(reading(orbiter.simulation, orbiter.kestrel, "nav/speed_m_s"), norm(state.velocity_m_s));
    EXPECT_DOUBLE_EQ(reading(orbiter.simulation, orbiter.kestrel, "nav/vertical_speed_m_s"),
                     dot(state.velocity_m_s, state.position_m) / norm(state.position_m));
}

TEST(Bubble, OnlyTheActiveVesselAtLowWarpIsInIt) {
    Orbiter orbiter = make_orbiter();
    Simulation& simulation = orbiter.simulation;
    run(simulation, 0.0, 1.0);
    EXPECT_FALSE(simulation.in_bubble(orbiter.kestrel));
    ASSERT_TRUE(simulation.set_active_vessel(orbiter.kestrel).has_value());
    run(simulation, 1.0, 2.0);
    EXPECT_TRUE(simulation.in_bubble(orbiter.kestrel));
    EXPECT_EQ(simulation.active_vessel(), orbiter.kestrel);

    // Above physics warp it goes back on rails, and returns when the warp comes down.
    simulation.time_warp().request_level(helios::sim::k_real_time_level + 3); // 1000×
    ASSERT_TRUE(simulation.advance(1.0 / 60.0).has_value());
    EXPECT_FALSE(simulation.in_bubble(orbiter.kestrel));
    simulation.time_warp().request_level(helios::sim::k_real_time_level);
    ASSERT_TRUE(simulation.advance(1.0 / 60.0).has_value());
    EXPECT_TRUE(simulation.in_bubble(orbiter.kestrel));

    ASSERT_TRUE(simulation.set_active_vessel(std::nullopt).has_value());
    ASSERT_TRUE(simulation.advance(1.0 / 60.0).has_value());
    EXPECT_FALSE(simulation.in_bubble(orbiter.kestrel));
    EXPECT_EQ(simulation.set_active_vessel(VesselId{42}).error().code, ErrorCode::OutOfRange);
}

// Coasting in the bubble changes nothing about the orbit: gravity stays with the propagator.
TEST(Bubble, ACoastingVesselFollowsExactlyTheSameOrbit) {
    Orbiter in_bubble = make_orbiter();
    Orbiter on_rails = make_orbiter();
    ASSERT_TRUE(in_bubble.simulation.set_active_vessel(in_bubble.kestrel).has_value());
    run(in_bubble.simulation, 0.0, 60.0);
    run(on_rails.simulation, 0.0, 60.0);
    ASSERT_TRUE(in_bubble.simulation.in_bubble(in_bubble.kestrel));
    EXPECT_EQ(in_bubble.vessel().state.state_in_domain.position_m,
              on_rails.vessel().state.state_in_domain.position_m);
    EXPECT_EQ(in_bubble.vessel().state.state_in_domain.velocity_m_s,
              on_rails.vessel().state.state_in_domain.velocity_m_s);
}

TEST(Bubble, ReactionWheelsTurnTheVesselAsEulerSays) {
    Orbiter orbiter = make_orbiter();
    Simulation& simulation = orbiter.simulation;
    ASSERT_TRUE(simulation.set_active_vessel(orbiter.kestrel).has_value());
    command(simulation, orbiter.kestrel, "guidance/hold", 0.0);
    run(simulation, 0.0, 1.0);
    const Vector3 nose_before = orbiter.nose();
    EXPECT_LT(norm(orbiter.vessel().attitude.angular_velocity_rad_s), 1e-9);

    // Full pitch for ten seconds: 1.5 kN m about the vessel's y axis.
    command(simulation, orbiter.kestrel, "attitude/pitch", 1.0);
    run(simulation, 1.0, 11.0);
    const double inertia_kg_m2 =
        orbiter.vessel().systems->mass_properties(simulation.now()).value().inertia_kg_m2(1, 1);
    const Vector3 rate_rad_s = rotate(helios::math::conjugate(orbiter.vessel().attitude.orientation),
                                      orbiter.vessel().attitude.angular_velocity_rad_s);
    EXPECT_NEAR(rate_rad_s.y, 1'500.0 * 10.0 / inertia_kg_m2, 0.01 * 1'500.0 * 10.0 / inertia_kg_m2);
    EXPECT_NEAR(rate_rad_s.x, 0.0, 1e-6);
    EXPECT_NEAR(rate_rad_s.z, 0.0, 1e-6);
    // ½ α t² of turn, and the wheel drew 200 W for it.
    EXPECT_NEAR(std::acos(dot(nose_before, orbiter.nose())), 0.5 * rate_rad_s.y * 10.0, 2e-3);
    EXPECT_NEAR(reading(simulation, orbiter.kestrel, "probe/charge_fraction"), 1.0 - 2'000.0 / 3.6e6, 1e-6);

    // With the wheels released the vessel keeps turning: nothing in space stops it.
    ASSERT_TRUE(
        simulation.release_command(orbiter.kestrel, "attitude/pitch", ControlSource::Pilot).has_value());
    run(simulation, 11.0, 13.0);
    const Vector3 rate_later_rad_s = rotate(helios::math::conjugate(orbiter.vessel().attitude.orientation),
                                            orbiter.vessel().attitude.angular_velocity_rad_s);
    EXPECT_NEAR(rate_later_rad_s.y, rate_rad_s.y, 1e-6);
}

TEST(Bubble, TheAttitudeHoldBringsTheNoseToTheCommandedPointing) {
    Orbiter orbiter = make_orbiter();
    Simulation& simulation = orbiter.simulation;
    // Drop the booster first: the upper stage alone turns in reasonable time.
    command(simulation, orbiter.kestrel, "staging/stage", 1.0);
    command(simulation, orbiter.kestrel, "staging/stage", 2.0);
    ASSERT_TRUE(simulation.set_active_vessel(orbiter.kestrel).has_value());
    run(simulation, 0.0, 1.0);
    const auto orbit_normal = [&] {
        const StateVector& state = orbiter.vessel().state.state_in_domain;
        const Vector3 normal = cross(state.position_m, state.velocity_m_s);
        return normal / norm(normal);
    };
    // It starts on the pointing it had on rails: prograde.
    const StateVector& state = orbiter.vessel().state.state_in_domain;
    EXPECT_GT(dot(orbiter.nose(), state.velocity_m_s / norm(state.velocity_m_s)), 0.9999);

    // Point along the orbit's normal: a quarter turn.
    command(simulation, orbiter.kestrel, "guidance/x", 0.0);
    command(simulation, orbiter.kestrel, "guidance/y", 1.0);
    run(simulation, 1.0, 2.0);
    EXPECT_LT(dot(orbiter.nose(), orbit_normal()), 0.2) << "it takes time: the wheels are not instant";
    run(simulation, 2.0, 61.0);
    EXPECT_GT(dot(orbiter.nose(), orbit_normal()), std::cos(0.5 * k_degree_rad));
    EXPECT_LT(norm(orbiter.vessel().attitude.angular_velocity_rad_s), 2e-3);
    EXPECT_LT(reading(simulation, orbiter.kestrel, "probe/charge_fraction"), 1.0);
}

// A burn flown in the bubble, with the attitude hold on prograde, ends up where the same burn
// on rails does: the thrust reaches the propagator as impulses in the middle of each tick.
TEST(Bubble, ABurnInTheBubbleMatchesTheSameBurnOnRails) {
    Orbiter in_bubble = make_orbiter();
    Orbiter on_rails = make_orbiter();
    ASSERT_TRUE(in_bubble.simulation.set_active_vessel(in_bubble.kestrel).has_value());
    for (Orbiter* orbiter : {&in_bubble, &on_rails}) {
        command(orbiter->simulation, orbiter->kestrel, "engine/throttle", 1.0);
        command(orbiter->simulation, orbiter->kestrel, "staging/stage", 1.0);
        run(orbiter->simulation, 0.0, 30.0);
        command(orbiter->simulation, orbiter->kestrel, "engine/throttle", 0.0);
        run(orbiter->simulation, 30.0, 40.0);
    }
    ASSERT_TRUE(in_bubble.simulation.in_bubble(in_bubble.kestrel));
    const StateVector& flown = in_bubble.vessel().state.state_in_domain;
    const StateVector& planned = on_rails.vessel().state.state_in_domain;
    // 30 s of 300 kN on 25 t: 375 m/s. The two agree to a small part of that.
    EXPECT_GT(norm(flown.velocity_m_s) - norm(k_low_orbit.velocity_m_s), 300.0);
    EXPECT_LT(norm(flown.velocity_m_s - planned.velocity_m_s), 0.5);
    EXPECT_LT(norm(flown.position_m - planned.position_m), 20.0);
    EXPECT_NEAR(reading(in_bubble.simulation, in_bubble.kestrel, "vessel/mass_kg"),
                k_kestrel_mass_kg - 30.0 * 98.0, 1e-6);
}

// Leaving the bubble in the middle of a burn hands the thrust in effect back to the rails.
TEST(Bubble, LeavingItDuringABurnKeepsTheBurnGoing) {
    Orbiter orbiter = make_orbiter();
    Simulation& simulation = orbiter.simulation;
    ASSERT_TRUE(simulation.set_active_vessel(orbiter.kestrel).has_value());
    command(simulation, orbiter.kestrel, "engine/throttle", 1.0);
    command(simulation, orbiter.kestrel, "staging/stage", 1.0);
    run(simulation, 0.0, 10.0);
    EXPECT_FALSE(orbiter.vessel().propagator.is_thrusting()) << "in the bubble the thrust is not planned";
    const double speed_in_bubble_m_s = norm(orbiter.vessel().state.state_in_domain.velocity_m_s);

    ASSERT_TRUE(simulation.set_active_vessel(std::nullopt).has_value());
    run(simulation, 10.0, 20.0);
    EXPECT_FALSE(simulation.in_bubble(orbiter.kestrel));
    EXPECT_TRUE(orbiter.vessel().propagator.is_thrusting());
    // Another ten seconds at about 12.3 m/s² (the vessel is lighter by then).
    EXPECT_NEAR(norm(orbiter.vessel().state.state_in_domain.velocity_m_s) - speed_in_bubble_m_s, 127.0, 3.0);
    // The wheels do not run on rails.
    EXPECT_EQ(reading(simulation, orbiter.kestrel, "attitude/pitch"), 0.0);
}

TEST(Bubble, ALandedVesselStaysOnItsSpotAsTheBodyTurns) {
    Lander lander = make_lander();
    Simulation& simulation = lander.simulation;
    EXPECT_EQ(lander.vessel().status, VesselStatus::Landed);
    EXPECT_DOUBLE_EQ(reading(simulation, lander.heron, "vessel/mass_kg"), k_heron_mass_kg);
    EXPECT_GT(lander.uprightness(), 1.0 - 1e-12);
    // Standing on its legs: the centre of mass a couple of metres up.
    const double height_m = lander.altitude_m();
    EXPECT_GT(height_m, 1.5);
    EXPECT_LT(height_m, 4.0);
    const helios::sim::LandedPlace place = *lander.vessel().landed;
    const Vector3 position_before_m = lander.vessel().state.state_in_domain.position_m;

    // Six days at any warp: a quarter of a lunar turn or so, on the same spot.
    ASSERT_TRUE(simulation.advance_to(at(6.0 * 86'400.0)).has_value());
    EXPECT_EQ(lander.vessel().status, VesselStatus::Landed);
    EXPECT_EQ(lander.vessel().landed->position_m, place.position_m);
    EXPECT_NEAR(lander.altitude_m(), height_m, 1e-6);
    EXPECT_GT(lander.uprightness(), 1.0 - 1e-12);
    EXPECT_GT(norm(lander.vessel().state.state_in_domain.position_m - position_before_m), 1.0e6);
    // It moves with the ground: about 4.6 m/s at the lunar equator.
    EXPECT_NEAR(norm(lander.vessel().state.state_in_domain.velocity_m_s), 4.62, 0.05);
    EXPECT_FALSE(simulation.warp_limit().has_value());
}

TEST(Bubble, ALanderHopsAndLandsAgain) {
    Lander lander = make_lander();
    Simulation& simulation = lander.simulation;
    ASSERT_TRUE(simulation.set_active_vessel(lander.heron).has_value());
    run(simulation, 0.0, 1.0);
    EXPECT_EQ(lander.vessel().status, VesselStatus::Landed);
    EXPECT_FALSE(simulation.in_bubble(lander.heron)) << "a landed vessel is pinned, not simulated";
    const double height_m = lander.altitude_m();
    const Vector3 place_before_m = lander.vessel().landed->position_m;

    // 9.6 kN against a lunar weight of 7.9 kN: it rises at 0.36 m/s².
    command(simulation, lander.heron, "engine/throttle", 0.6);
    command(simulation, lander.heron, "staging/stage", 1.0);
    run(simulation, 1.0, 4.0);
    EXPECT_EQ(lander.vessel().status, VesselStatus::Flying);
    EXPECT_TRUE(simulation.in_bubble(lander.heron));
    EXPECT_NEAR(lander.altitude_m() - height_m, 0.5 * 0.36 * 9.0, 0.3);
    EXPECT_GT(lander.uprightness(), 0.999);

    // Engine off: up a little more, then down from about two metres, at some 2.6 m/s.
    command(simulation, lander.heron, "engine/throttle", 0.0);
    run(simulation, 4.0, 12.0);
    EXPECT_EQ(lander.vessel().status, VesselStatus::Landed);
    EXPECT_FALSE(simulation.in_bubble(lander.heron));
    EXPECT_NEAR(lander.altitude_m(), height_m, 0.05);
    EXPECT_GT(lander.uprightness(), 0.999);
    EXPECT_LT(norm(lander.vessel().landed->position_m - place_before_m), 1.0);
    EXPECT_NEAR(reading(simulation, lander.heron, "vessel/mass_kg"), k_heron_mass_kg - 3.0 * 0.6 * 5.26, 0.1);
}

TEST(Bubble, FallingTooFarDestroysTheVessel) {
    Lander lander = make_lander();
    Simulation& simulation = lander.simulation;
    ASSERT_TRUE(simulation.set_active_vessel(lander.heron).has_value());
    command(simulation, lander.heron, "engine/throttle", 1.0);
    command(simulation, lander.heron, "staging/stage", 1.0);
    // Ten seconds at full thrust: 84 m up and climbing at 17 m/s. Then nothing.
    run(simulation, 0.0, 10.0);
    EXPECT_GT(lander.altitude_m(), 60.0);
    command(simulation, lander.heron, "engine/throttle", 0.0);
    run(simulation, 10.0, 45.0);
    EXPECT_EQ(lander.vessel().status, VesselStatus::Crashed);
    EXPECT_FALSE(simulation.in_bubble(lander.heron));
    EXPECT_LT(lander.altitude_m(), 5.0);
    EXPECT_EQ(simulation.command(lander.heron, "engine/throttle", ControlSource::Pilot, 1.0).error().code,
              ErrorCode::InvalidArgument);
}

TEST(Bubble, RejectsImpossibleLandingSites) {
    Lander lander = make_lander();
    const helios::vessel::PartCatalog catalog = helios::test::load_stock_parts();
    EXPECT_EQ(lander.simulation
                  .add_landed_vessel("nowhere", BodyId{999}, 0.0, 0.0,
                                     helios::test::make_demo_vessel(catalog, "Heron", Epoch{}))
                  .error()
                  .code,
              ErrorCode::InvalidArgument);
    EXPECT_EQ(lander.simulation
                  .add_landed_vessel("lost", lander.moon, std::nan(""), 0.0,
                                     helios::test::make_demo_vessel(catalog, "Heron", Epoch{}))
                  .error()
                  .code,
              ErrorCode::NotFinite);
    EXPECT_EQ(lander.simulation
                  .add_landed_vessel("late", lander.moon, 0.0, 0.0,
                                     helios::test::make_demo_vessel(catalog, "Heron", at(100.0)))
                  .error()
                  .code,
              ErrorCode::OutOfRange);
}

} // namespace
