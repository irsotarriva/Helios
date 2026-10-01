#include "helios/sim/scene_snapshot.hpp"
#include "helios/sim/simulation.hpp"

#include <gtest/gtest.h>
#include <memory>

#include "../support/universes.hpp"

namespace {

using helios::bodies::BodyId;
using helios::dynamics::EnckePropagator;
using helios::dynamics::EventKind;
using helios::dynamics::VesselState;
using helios::math::norm;
using helios::math::Vector3;
using helios::orbital::StateVector;
using helios::sim::build_snapshot;
using helios::sim::Focus;
using helios::sim::k_real_time_level;
using helios::sim::k_warp_levels;
using helios::sim::LineKind;
using helios::sim::SceneSnapshot;
using helios::sim::Simulation;
using helios::sim::SnapshotExchange;
using helios::sim::VesselId;
using helios::time::Epoch;
using helios::time::seconds_between;

constexpr double k_frame_s = 1.0 / 60.0;

[[nodiscard]] Epoch at(double seconds) {
    return Epoch::from_seconds(seconds).value();
}

struct Fixture {
    Simulation simulation;
    BodyId earth;
    BodyId moon;
};

[[nodiscard]] Fixture make_fixture() {
    helios::test::EarthMoonCr3bp system = helios::test::make_earth_moon_cr3bp();
    const BodyId earth = system.earth;
    const BodyId moon = system.moon;
    return Fixture{
        .simulation =
            Simulation::make(std::move(system.tree), std::move(system.catalog), Epoch{}, {}).value(),
        .earth = earth,
        .moon = moon};
}

// Leaves the Moon's sphere of influence about a day after the start.
const StateVector k_escape{.position_m = {6.7e6, 0.0, 0.0}, .velocity_m_s = {3'000.0, 500.0, 0.0}};

TEST(Simulation, WarpSlowsForTheDomainExitAndTheTrajectoryIsTheWarpFreeOne) {
    Fixture fixture = make_fixture();
    Simulation& simulation = fixture.simulation;
    const VesselId vessel = simulation.add_vessel("probe", k_escape, fixture.moon).value();
    const auto limit = simulation.warp_limit();
    ASSERT_TRUE(limit.has_value());
    EXPECT_EQ(limit->kind, EventKind::DomainExit);

    simulation.time_warp().request_level(k_warp_levels.size() - 1);
    double slowest_near_event = 1e9;
    int frames = 0;
    while (simulation.vessel(vessel).value().get().state.domain == fixture.moon && frames < 100'000) {
        ASSERT_TRUE(simulation.advance(k_frame_s).has_value());
        const double to_event_s = seconds_between(simulation.now(), limit->epoch);
        if (to_event_s > 0.0 && to_event_s < 60.0) {
            slowest_near_event = std::min(slowest_near_event, simulation.effective_warp());
        }
        ++frames;
    }
    EXPECT_EQ(simulation.vessel(vessel).value().get().state.domain, fixture.earth);
    EXPECT_LE(slowest_near_event, 10.0) << "warp must drop well before the sphere-of-influence exit";
    // SOI changes do not cancel the requested warp; bursts and impacts do.
    EXPECT_EQ(simulation.time_warp().requested_factor(), k_warp_levels.back());

    // Same trajectory as a propagator driven directly, bit for bit (D14).
    const Fixture reference = make_fixture();
    EnckePropagator direct = EnckePropagator::make(reference.simulation.gravity(),
                                                   VesselState{fixture.moon, Epoch{}, k_escape}, {})
                                 .value();
    const auto expected = direct.state_at(simulation.now()).value();
    EXPECT_EQ(expected.state_in_domain.position_m,
              simulation.vessel(vessel).value().get().state.state_in_domain.position_m);
}

TEST(Simulation, WarpStopsExactlyAtABurnAndDropsToRealTime) {
    Fixture fixture = make_fixture();
    Simulation& simulation = fixture.simulation;
    const StateVector geo{.position_m = {4.2e7, 0.0, 0.0}, .velocity_m_s = {0.0, 3'080.0, 0.0}};
    const VesselId vessel = simulation.add_vessel("sat", geo, fixture.earth).value();
    const Epoch burn = at(20'000.25);
    ASSERT_TRUE(simulation.schedule_impulse(vessel, {burn, {0.0, 50.0, 0.0}}).has_value());
    simulation.time_warp().request_level(k_warp_levels.size() - 1);
    bool landed = false;
    for (int frame = 0; frame < 10'000 && simulation.now() < burn; ++frame) {
        ASSERT_TRUE(simulation.advance(k_frame_s).has_value());
        landed = landed || simulation.now() == burn;
    }
    EXPECT_TRUE(landed);
    EXPECT_EQ(simulation.time_warp().requested_level(), k_real_time_level);
    EXPECT_EQ(simulation.vessel(vessel).value().get().propagator.statistics().impulses, 1U);
}

// The top warp level is only usable because a low orbit is analytic (BRIEFING §5.3): two wall
// seconds at 1e9× are 63 years, some 340,000 revolutions.
TEST(Simulation, ALowOrbitSurvivesTheHighestWarpWithoutIntegrating) {
    Fixture fixture = make_fixture();
    Simulation& simulation = fixture.simulation;
    const StateVector low_orbit{.position_m = {7.0e6, 0.0, 0.0}, .velocity_m_s = {0.0, 7'546.0, 0.0}};
    const VesselId vessel = simulation.add_vessel("station", low_orbit, fixture.earth).value();
    simulation.time_warp().request_level(k_warp_levels.size() - 1);
    EXPECT_EQ(simulation.time_warp().requested_factor(), 1e9);
    for (int frame = 0; frame < 120; ++frame) {
        ASSERT_TRUE(simulation.advance(k_frame_s).has_value());
    }
    EXPECT_NEAR(seconds_between(Epoch{}, simulation.now()), 2.0e9, 1.0);
    const helios::sim::Vessel& station = simulation.vessels()[vessel.index];
    EXPECT_EQ(station.status, helios::sim::VesselStatus::Flying);
    EXPECT_TRUE(station.propagator.is_analytic());
    EXPECT_LE(station.propagator.statistics().accepted_steps, 2U);
    EXPECT_NEAR(norm(station.state.state_in_domain.position_m), 7.0e6, 5.0e3);
}

// An encounter beyond the event horizon when the vessel was added must still slow the warp once
// it comes within the horizon: predictions are refreshed as time passes, not only on events.
TEST(Simulation, AnEncounterBeyondTheEventHorizonIsFoundLaterAndLimitsWarp) {
    helios::test::EarthMoonCr3bp system = helios::test::make_earth_moon_cr3bp();
    const BodyId earth = system.earth;
    const BodyId moon = system.moon;
    constexpr double k_arrival_s = 2.0 * 86'400.0;
    // 40,000 km from the Moon and falling towards it two days from now, run back to the start.
    const auto moon_at_arrival =
        system.tree
            ->relative_state(system.catalog->body(moon)->get().frame,
                             system.catalog->body(earth)->get().frame, at(k_arrival_s))
            .value();
    const StateVector arrival{.position_m = moon_at_arrival.position_m + Vector3{-4.0e7, 0.0, 0.0},
                              .velocity_m_s = moon_at_arrival.velocity_m_s + Vector3{1'200.0, 300.0, 0.0}};
    const StateVector start = helios::orbital::propagate_conic(
                                  arrival, helios::test::EarthMoonCr3bp::k_earth_mu_m3_s2, -k_arrival_s)
                                  .value();
    helios::sim::SimulationOptions options;
    options.event_horizon_s = 86'400.0;
    Simulation simulation =
        Simulation::make(std::move(system.tree), std::move(system.catalog), Epoch{}, options).value();
    const VesselId vessel = simulation.add_vessel("probe", start, earth).value();
    ASSERT_FALSE(simulation.warp_limit().has_value()) << "the entry is beyond the horizon at the start";

    simulation.time_warp().request_level(k_warp_levels.size() - 1);
    double slowest = 1e9;
    int frames = 0;
    while (simulation.vessel(vessel).value().get().state.domain == earth && frames < 100'000) {
        ASSERT_TRUE(simulation.advance(k_frame_s).has_value());
        slowest = std::min(slowest, simulation.effective_warp());
        ++frames;
    }
    EXPECT_EQ(simulation.vessel(vessel).value().get().state.domain, moon);
    EXPECT_LE(slowest, 10.0) << "warp must drop before the sphere-of-influence entry";
    EXPECT_LT(seconds_between(Epoch{}, simulation.now()), k_arrival_s);
}

TEST(Simulation, MaximumWarpIsAnOption) {
    helios::test::EarthMoonCr3bp system = helios::test::make_earth_moon_cr3bp();
    helios::sim::SimulationOptions options;
    options.max_warp_factor = 1e6;
    Simulation simulation =
        Simulation::make(std::move(system.tree), std::move(system.catalog), Epoch{}, options).value();
    simulation.time_warp().request_level(k_warp_levels.size() - 1);
    EXPECT_EQ(simulation.time_warp().requested_factor(), 1e6);
    EXPECT_EQ(build_snapshot(simulation, Focus::body(BodyId{0})).value().max_warp_factor, 1e6);
}

TEST(Simulation, ReachingTheSurfaceMarksTheVesselCrashed) {
    Fixture fixture = make_fixture();
    Simulation& simulation = fixture.simulation;
    const StateVector falling{.position_m = {6.6e6, 0.0, 0.0}, .velocity_m_s = {-500.0, 1'000.0, 0.0}};
    const VesselId vessel = simulation.add_vessel("lawn dart", falling, fixture.earth).value();
    // One huge step: the end state is back above ground on the far side, the impact is not.
    ASSERT_TRUE(simulation.advance_to(at(3'600.0)).has_value());
    const helios::sim::Vessel& crashed = simulation.vessels()[vessel.index];
    EXPECT_EQ(crashed.status, helios::sim::VesselStatus::Crashed);
    EXPECT_NEAR(norm(crashed.state.state_in_domain.position_m), 6.371e6, 1.0);
    EXPECT_LT(seconds_between(Epoch{}, crashed.state.epoch), 3'600.0);
}

TEST(SceneSnapshot, PositionsAreRelativeToTheFocusAndLinesAreBuilt) {
    Fixture fixture = make_fixture();
    Simulation& simulation = fixture.simulation;
    const VesselId vessel = simulation.add_vessel("probe", k_escape, fixture.moon).value();
    ASSERT_TRUE(simulation.advance_to(at(600.0)).has_value());

    const SceneSnapshot on_earth = build_snapshot(simulation, Focus::body(fixture.earth)).value();
    EXPECT_EQ(on_earth.focus_name, "Earth");
    ASSERT_EQ(on_earth.bodies.size(), 2U);
    EXPECT_EQ(on_earth.bodies[fixture.earth.index].position_m, Vector3{});
    const auto moon_in_earth =
        simulation.tree()
            .relative_state(simulation.catalog().body(fixture.moon)->get().frame,
                            simulation.catalog().body(fixture.earth)->get().frame, at(600.0))
            .value();
    EXPECT_EQ(on_earth.bodies[fixture.moon.index].position_m, moon_in_earth.position_m);

    int body_orbits = 0;
    int trajectories = 0;
    for (const auto& line : on_earth.lines) {
        if (line.kind == LineKind::BodyOrbit) {
            ++body_orbits;
            EXPECT_TRUE(line.closed);
            EXPECT_EQ(line.owner, fixture.moon.index);
        } else {
            ++trajectories;
        }
    }
    EXPECT_EQ(body_orbits, 1);
    EXPECT_GE(trajectories, 1);
    const auto& first_trajectory =
        *std::ranges::find(on_earth.lines, LineKind::VesselTrajectory, &helios::sim::LineView::kind);
    EXPECT_EQ(first_trajectory.points_m.front(), on_earth.vessels.front().position_m);

    const SceneSnapshot on_vessel = build_snapshot(simulation, Focus::vessel(vessel)).value();
    EXPECT_EQ(on_vessel.vessels.front().position_m, Vector3{});
    EXPECT_LT(norm(on_vessel.bodies[fixture.moon.index].position_m + k_escape.position_m), 1e7);
    ASSERT_TRUE(on_vessel.next_event.has_value());
    EXPECT_EQ(on_vessel.next_event->kind, EventKind::DomainExit);

    SnapshotExchange exchange;
    EXPECT_EQ(exchange.latest(), nullptr);
    exchange.publish(std::make_shared<const SceneSnapshot>(on_vessel));
    EXPECT_EQ(exchange.latest()->focus_name, "probe");
}

} // namespace
