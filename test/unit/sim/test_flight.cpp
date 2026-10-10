#include "helios/sim/scene_snapshot.hpp"
#include "helios/sim/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "../support/universes.hpp"
#include "../vessel/support.hpp"

// Vessels made of parts in the simulation: commands on the control bus become thrust on the
// trajectory, and staging turns one vessel into two.
namespace {

using helios::bodies::BodyId;
using helios::core::ErrorCode;
using helios::math::norm;
using helios::orbital::StateVector;
using helios::sim::k_real_time_level;
using helios::sim::k_warp_levels;
using helios::sim::Simulation;
using helios::sim::Vessel;
using helios::sim::VesselId;
using helios::time::Epoch;
using helios::vessel::ControlSource;

constexpr double k_kestrel_mass_kg = 25'490.0;
constexpr double k_upper_stage_mass_kg = 4'830.0;
constexpr double k_booster_burn_s = 18'000.0 / 98.0;

// A 630 km circular orbit.
const StateVector k_low_orbit{.position_m = {7.0e6, 0.0, 0.0}, .velocity_m_s = {0.0, 7'546.0, 300.0}};

[[nodiscard]] Epoch at(double seconds) {
    return Epoch::from_seconds(seconds).value();
}

struct Flight {
    Simulation simulation;
    VesselId kestrel;
    BodyId earth;

    [[nodiscard]] const Vessel& vessel(VesselId id) const { return simulation.vessel(id).value().get(); }
    [[nodiscard]] double reading(VesselId id, std::string_view signal) const {
        const helios::vessel::ControlBus& bus = vessel(id).systems->bus();
        return bus.value(bus.find(signal).value());
    }
    void command(std::string_view signal, double value) {
        const auto commanded = simulation.command(kestrel, signal, ControlSource::Pilot, value);
        ASSERT_TRUE(commanded.has_value()) << helios::core::describe(commanded.error());
    }
    void schedule(double epoch_s, std::string_view signal, double value) {
        const auto scheduled = simulation.schedule_command(kestrel, {.epoch = at(epoch_s),
                                                                     .signal = std::string{signal},
                                                                     .source = ControlSource::Pilot,
                                                                     .value = value});
        ASSERT_TRUE(scheduled.has_value()) << helios::core::describe(scheduled.error());
    }
};

[[nodiscard]] Flight make_flight() {
    helios::test::EarthMoonCr3bp system = helios::test::make_earth_moon_cr3bp();
    const BodyId earth = system.earth;
    Simulation simulation =
        Simulation::make(std::move(system.tree), std::move(system.catalog), Epoch{}, {}).value();
    const helios::vessel::PartCatalog catalog = helios::test::load_stock_parts();
    const VesselId kestrel = simulation
                                 .add_vessel("Kestrel", k_low_orbit, earth,
                                             helios::test::make_demo_vessel(catalog, "Kestrel", Epoch{}))
                                 .value();
    return Flight{.simulation = std::move(simulation), .kestrel = kestrel, .earth = earth};
}

TEST(Flight, ThrottleAndStagingBurnPropellantAndChangeTheOrbit) {
    Flight flight = make_flight();
    Simulation& simulation = flight.simulation;
    EXPECT_DOUBLE_EQ(flight.reading(flight.kestrel, "vessel/mass_kg"), k_kestrel_mass_kg);
    const double initial_speed_m_s = norm(k_low_orbit.velocity_m_s);

    flight.command("engine/throttle", 1.0);
    flight.command("staging/stage", 1.0);
    EXPECT_TRUE(flight.vessel(flight.kestrel).propagator.pending_thrust_changes().size() >= 2U);
    ASSERT_TRUE(simulation.advance_to(at(100.0)).has_value());
    EXPECT_TRUE(flight.vessel(flight.kestrel).propagator.is_thrusting());
    EXPECT_DOUBLE_EQ(flight.reading(flight.kestrel, "vessel/thrust_n"), 300'000.0);
    EXPECT_DOUBLE_EQ(flight.reading(flight.kestrel, "vessel/mass_kg"), k_kestrel_mass_kg - 9'800.0);
    EXPECT_DOUBLE_EQ(flight.reading(flight.kestrel, "lower_engine/chamber_temperature_k"), 3'650.0);

    // The booster burns out by itself; prograde thrust has raised the speed by most of the
    // 3749 m/s the rocket equation gives (the rest went into climbing).
    ASSERT_TRUE(simulation.advance_to(at(k_booster_burn_s + 10.0)).has_value());
    const Vessel& kestrel = flight.vessel(flight.kestrel);
    EXPECT_FALSE(kestrel.propagator.is_thrusting());
    EXPECT_EQ(flight.reading(flight.kestrel, "vessel/thrust_n"), 0.0);
    EXPECT_EQ(flight.reading(flight.kestrel, "lower_tank/lox_kg"), 0.0);
    const double gained_m_s = norm(kestrel.state.state_in_domain.velocity_m_s) - initial_speed_m_s;
    EXPECT_GT(gained_m_s, 3'000.0);
    EXPECT_LT(gained_m_s, 3'749.0);
    EXPECT_EQ(kestrel.propagator.statistics().thrust_changes, 2U);
}

TEST(Flight, StagingMakesTheDroppedStageAVesselAndPushesTheTwoApart) {
    Flight flight = make_flight();
    Simulation& simulation = flight.simulation;
    ASSERT_TRUE(simulation.advance_to(at(50.0)).has_value());
    flight.command("staging/stage", 1.0); // booster lit, but the throttle is closed
    flight.command("staging/stage", 2.0);

    ASSERT_EQ(simulation.vessels().size(), 2U);
    const VesselId booster{1};
    EXPECT_EQ(flight.vessel(booster).name, "Kestrel (decoupler)");
    EXPECT_EQ(flight.vessel(flight.kestrel).systems->assembly().parts().size(), 3U);
    EXPECT_DOUBLE_EQ(flight.reading(flight.kestrel, "vessel/mass_kg"), k_upper_stage_mass_kg);
    EXPECT_DOUBLE_EQ(flight.reading(booster, "vessel/mass_kg"), k_kestrel_mass_kg - k_upper_stage_mass_kg);
    // The booster's centre of mass is where its parts are: behind the upper stage's, along the
    // nose (prograde).
    const StateVector kept = flight.vessel(flight.kestrel).state.state_in_domain;
    const helios::math::Vector3 behind_m =
        kept.position_m - flight.vessel(booster).state.state_in_domain.position_m;
    EXPECT_GT(norm(behind_m), 2.0);
    EXPECT_LT(norm(behind_m), 30.0);
    EXPECT_NEAR(helios::math::dot(behind_m, kept.velocity_m_s) / (norm(behind_m) * norm(kept.velocity_m_s)),
                1.0, 1e-9);

    // 2500 N s between 4.83 t and 20.66 t: 0.518 + 0.121 m/s apart, and the centre of mass of
    // the pair carries on as before.
    ASSERT_TRUE(simulation.advance_to(at(50.001)).has_value());
    const StateVector upper = flight.vessel(flight.kestrel).state.state_in_domain;
    const StateVector lower = flight.vessel(booster).state.state_in_domain;
    const double booster_mass_kg = k_kestrel_mass_kg - k_upper_stage_mass_kg;
    EXPECT_NEAR(norm(upper.velocity_m_s - lower.velocity_m_s),
                2'500.0 / k_upper_stage_mass_kg + 2'500.0 / booster_mass_kg, 1e-6);
    EXPECT_GT(norm(upper.velocity_m_s), norm(lower.velocity_m_s)); // the upper stage is ahead
    Simulation undisturbed = make_flight().simulation;
    ASSERT_TRUE(undisturbed.advance_to(at(50.001)).has_value());
    const helios::math::Vector3 centre_velocity_m_s =
        (k_upper_stage_mass_kg * upper.velocity_m_s + booster_mass_kg * lower.velocity_m_s)
        / k_kestrel_mass_kg;
    EXPECT_LT(norm(centre_velocity_m_s - undisturbed.vessels()[0].state.state_in_domain.velocity_m_s), 1e-7);

    // Each side now answers for itself.
    flight.command("engine/throttle", 1.0);
    ASSERT_TRUE(simulation.advance_to(at(60.0)).has_value());
    EXPECT_DOUBLE_EQ(flight.reading(flight.kestrel, "vessel/thrust_n"), 30'000.0);
    EXPECT_DOUBLE_EQ(flight.reading(booster, "vessel/thrust_n"), 0.0);
    EXPECT_EQ(
        simulation.command(flight.kestrel, "lower_engine/throttle", ControlSource::Pilot, 1.0).error().code,
        ErrorCode::InvalidArgument);
}

// D14 for a whole flight: a programme of commands flown at one frame per simulated second and
// in three frames gives the same vessels in the same places, bit for bit.
TEST(Flight, AProgrammedFlightIsIdenticalAtAnyFrameLength) {
    const auto fly = [](double frame_s) {
        Flight flight = make_flight();
        flight.command("engine/throttle", 1.0);
        flight.schedule(60.0, "staging/stage", 1.0);
        flight.schedule(300.0, "staging/stage", 2.0);
        flight.schedule(420.5, "engine/throttle", 0.5);
        flight.schedule(500.25, "guidance/y", 1.0); // turn out of the plane, half-way
        flight.schedule(650.0, "engine/throttle", 0.0);
        constexpr double k_end_s = 3'000.0;
        const int frames = static_cast<int>(std::ceil(k_end_s / frame_s));
        for (int frame = 1; frame <= frames; ++frame) {
            EXPECT_TRUE(flight.simulation.advance_to(at(std::min(frame * frame_s, k_end_s))).has_value());
        }
        return flight;
    };
    const Flight fine = fly(1.0);
    const Flight coarse = fly(1'234.5);
    ASSERT_EQ(fine.simulation.vessels().size(), 2U);
    ASSERT_EQ(coarse.simulation.vessels().size(), 2U);
    for (const VesselId id : {VesselId{0}, VesselId{1}}) {
        EXPECT_EQ(fine.vessel(id).state.state_in_domain.position_m,
                  coarse.vessel(id).state.state_in_domain.position_m)
            << id.index;
        EXPECT_EQ(fine.vessel(id).state.state_in_domain.velocity_m_s,
                  coarse.vessel(id).state.state_in_domain.velocity_m_s)
            << id.index;
        EXPECT_EQ(fine.reading(id, "vessel/mass_kg"), coarse.reading(id, "vessel/mass_kg")) << id.index;
    }
    // Booster burn, 120.5 s at full and 229.5 s at half throttle on the upper stage.
    EXPECT_DOUBLE_EQ(fine.reading(fine.kestrel, "vessel/mass_kg"),
                     k_upper_stage_mass_kg - 9.0 * 120.5 - 4.5 * 229.5);
    EXPECT_FALSE(fine.vessel(fine.kestrel).propagator.is_thrusting());
}

TEST(Flight, WarpStopsForAPlannedIgnitionAndThePathShowsTheBurn) {
    Flight flight = make_flight();
    Simulation& simulation = flight.simulation;
    const auto farthest_predicted_m = [&] {
        double farthest_m = 0.0;
        for (const auto& segment : flight.vessel(flight.kestrel).prediction) {
            for (const auto& sample : segment.samples) {
                farthest_m = std::max(farthest_m, norm(sample.position_m));
            }
        }
        return farthest_m;
    };
    EXPECT_LT(farthest_predicted_m(), 7.2e6);
    EXPECT_FALSE(simulation.warp_limit().has_value());

    flight.command("engine/throttle", 1.0);
    flight.schedule(3'600.0, "staging/stage", 1.0);
    // The predicted path already includes the burn: the booster takes the vessel far out.
    EXPECT_GT(farthest_predicted_m(), 1.0e7);
    const auto limit = simulation.warp_limit();
    ASSERT_TRUE(limit.has_value());
    EXPECT_FALSE(limit->kind.has_value());
    EXPECT_EQ(limit->epoch, at(3'600.0));

    simulation.time_warp().request_level(k_warp_levels.size() - 1);
    for (int frame = 0; frame < 100'000 && simulation.now() < at(3'600.0); ++frame) {
        ASSERT_TRUE(simulation.advance(1.0 / 60.0).has_value());
    }
    // It lands exactly on the ignition and drops to real time for it.
    EXPECT_EQ(simulation.now(), at(3'600.0));
    EXPECT_EQ(simulation.time_warp().requested_level(), k_real_time_level);
    ASSERT_TRUE(simulation.advance(1.0 / 60.0).has_value());
    EXPECT_TRUE(flight.vessel(flight.kestrel).propagator.is_thrusting());
}

// What the map view draws: while coasting, the path ahead; under thrust, the orbit the vessel
// has at this instant (the one its apsis markers belong to) and, apart from it, the burn path.
TEST(Flight, UnderThrustTheSnapshotHasTheCurrentOrbitAndTheBurnPath) {
    Flight flight = make_flight();
    Simulation& simulation = flight.simulation;
    using helios::sim::LineKind;
    const auto lines_of = [&](LineKind kind) {
        const auto snapshot =
            helios::sim::build_snapshot(simulation, helios::sim::Focus::body(flight.earth)).value();
        std::vector<helios::sim::LineView> lines;
        for (const helios::sim::LineView& line : snapshot.lines) {
            if (line.kind == kind && line.owner == flight.kestrel.index) {
                lines.push_back(line);
            }
        }
        return lines;
    };
    EXPECT_EQ(lines_of(LineKind::VesselTrajectory).size(), 1U);
    EXPECT_TRUE(lines_of(LineKind::VesselOrbit).empty());
    EXPECT_TRUE(lines_of(LineKind::VesselBurnPath).empty());

    flight.command("engine/throttle", 1.0);
    flight.command("staging/stage", 1.0);
    ASSERT_TRUE(simulation.advance_to(at(30.0)).has_value());
    EXPECT_TRUE(lines_of(LineKind::VesselTrajectory).empty());
    const auto burn_path = lines_of(LineKind::VesselBurnPath);
    ASSERT_EQ(burn_path.size(), 1U);
    const auto orbit = lines_of(LineKind::VesselOrbit);
    ASSERT_EQ(orbit.size(), 1U);
    EXPECT_TRUE(orbit.front().closed); // still a bound orbit after 30 s of the booster
    // The orbit is the osculating one: it passes through the vessel and reaches its apoapsis.
    const Vessel& kestrel = flight.vessel(flight.kestrel);
    const auto conic = helios::orbital::conic_geometry(kestrel.state.state_in_domain, 3.986004354e14).value();
    double nearest_m = 1e30;
    double farthest_m = 0.0;
    for (const helios::math::Vector3& point_m : orbit.front().points_m) {
        nearest_m = std::min(nearest_m, norm(point_m - kestrel.state.state_in_domain.position_m));
        farthest_m = std::max(farthest_m, norm(point_m));
    }
    EXPECT_LT(nearest_m, 0.02 * norm(kestrel.state.state_in_domain.position_m));
    EXPECT_NEAR(farthest_m, conic.apoapsis_radius_m, 1e-3 * conic.apoapsis_radius_m);
    // The burn path goes where the whole booster burn leads: far beyond that orbit.
    double burn_farthest_m = 0.0;
    for (const helios::math::Vector3& point_m : burn_path.front().points_m) {
        burn_farthest_m = std::max(burn_farthest_m, norm(point_m));
    }
    EXPECT_GT(burn_farthest_m, 1.5 * farthest_m);

    // Engine off: one line again, the coasting path.
    flight.command("engine/throttle", 0.0);
    ASSERT_TRUE(simulation.advance_to(at(31.0)).has_value());
    EXPECT_EQ(lines_of(LineKind::VesselTrajectory).size(), 1U);
    EXPECT_TRUE(lines_of(LineKind::VesselOrbit).empty());
    EXPECT_TRUE(lines_of(LineKind::VesselBurnPath).empty());
}

TEST(Flight, RejectsCommandsThatCannotBeCarriedOut) {
    Flight flight = make_flight();
    Simulation& simulation = flight.simulation;
    const VesselId bare = simulation.add_vessel("bare", k_low_orbit, flight.earth).value();
    EXPECT_EQ(simulation.command(bare, "engine/throttle", ControlSource::Pilot, 1.0).error().code,
              ErrorCode::InvalidArgument);
    EXPECT_EQ(simulation.command(VesselId{99}, "engine/throttle", ControlSource::Pilot, 1.0).error().code,
              ErrorCode::OutOfRange);
    EXPECT_EQ(simulation.command(flight.kestrel, "engine/warp", ControlSource::Pilot, 1.0).error().code,
              ErrorCode::InvalidArgument);
    ASSERT_TRUE(simulation.advance_to(at(10.0)).has_value());
    EXPECT_EQ(
        simulation
            .schedule_command(flight.kestrel, {.epoch = at(5.0), .signal = "engine/throttle", .value = 1.0})
            .error()
            .code,
        ErrorCode::OutOfRange);
    EXPECT_EQ(
        simulation
            .schedule_command(flight.kestrel, {.epoch = at(50.0), .signal = "engine/warp", .value = 1.0})
            .error()
            .code,
        ErrorCode::InvalidArgument);
    // Systems that are ahead of the clock, or already burning, cannot be added.
    const helios::vessel::PartCatalog catalog = helios::test::load_stock_parts();
    EXPECT_EQ(simulation
                  .add_vessel("late", k_low_orbit, flight.earth,
                              helios::test::make_demo_vessel(catalog, "Kestrel", at(20.0)))
                  .error()
                  .code,
              ErrorCode::OutOfRange);
    helios::vessel::VesselSystems burning = helios::test::make_demo_vessel(catalog, "Kestrel", at(1.0));
    std::vector<helios::vessel::Separation> separations;
    ASSERT_TRUE(
        burning.command("engine/throttle", ControlSource::Pilot, 1.0, at(1.0), separations).has_value());
    ASSERT_TRUE(
        burning.command("staging/stage", ControlSource::Pilot, 1.0, at(1.0), separations).has_value());
    EXPECT_EQ(simulation.add_vessel("burning", k_low_orbit, flight.earth, std::move(burning)).error().code,
              ErrorCode::InvalidArgument);
    // Releasing the pilot's hold returns the signal to its default.
    flight.command("engine/throttle", 0.7);
    ASSERT_TRUE(
        simulation.release_command(flight.kestrel, "engine/throttle", ControlSource::Pilot).has_value());
    EXPECT_EQ(flight.reading(flight.kestrel, "engine/throttle"), 0.0);
}

} // namespace
