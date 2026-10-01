#include "helios/sim/solar_system.hpp"

#include <algorithm>
#include <gtest/gtest.h>
#include <optional>
#include <string_view>
#include <utility>

#include "scenario.hpp"
#include "simulation_host.hpp"

namespace {

using helios::time::Epoch;
using helios::time::seconds_between;

constexpr double k_day_s = 86'400.0;

[[nodiscard]] helios::sim::Simulation make_demo(double start_unix_s) {
    auto universe =
        helios::sim::load_stock_solar_system(HELIOS_DATA_DIR "/solar_system", std::nullopt).value();
    auto simulation =
        helios::sim::Simulation::make(std::move(universe.tree), std::move(universe.catalog),
                                      helios::app::epoch_from_unix_utc(start_unix_s).value(), {})
            .value();
    EXPECT_TRUE(helios::app::add_demo_vessels(simulation).has_value());
    return simulation;
}

TEST(Epochs, UnixTimeRoundTripsThroughTheCalendar) {
    // 2026-09-21 14:13:20 UTC.
    const Epoch epoch = helios::app::epoch_from_unix_utc(1'790'000'000.0).value();
    EXPECT_EQ(helios::app::format_epoch_utc(epoch), "2026-09-21 14:13:20 UTC");
    // 2020-01-01 00:00:00 UTC = 00:01:09.184 TT, 7,304.5 days after J2000.0 (12:00 TT).
    EXPECT_NEAR(seconds_between(Epoch{}, helios::app::epoch_from_unix_utc(1'577'836'800.0).value()),
                7'304.5 * k_day_s + 69.184, 1e-6);
}

TEST(Epochs, TheCalendarReachesFarBeyondTheRangeOfChronoYears) {
    // The Gregorian calendar repeats every 400 years = 146,097 days: 190 cycles later it is the
    // same date and time in the year 2026 + 76,000.
    constexpr double k_cycle_s = 146'097.0 * k_day_s;
    const Epoch far = helios::app::epoch_from_unix_utc(1'790'000'000.0 + 190.0 * k_cycle_s).value();
    EXPECT_EQ(helios::app::format_epoch_utc(far), "78026-09-21 14:13:20 UTC");
    EXPECT_EQ(helios::app::format_epoch_utc(helios::app::epoch_from_unix_utc(951'782'400.0).value()),
              "2000-02-29 00:00:00 UTC");
    EXPECT_EQ(helios::app::format_epoch_utc(helios::app::epoch_from_unix_utc(-1.0).value()),
              "1969-12-31 23:59:59 UTC");
}

// The reason for the warp levels above 1e6×: a probe leaving the Solar System. Two wall seconds
// at 1e9× are 63 years.
TEST(DemoScenario, InterstellarProbeCoastsAtTheHighestWarp) {
    auto universe =
        helios::sim::load_stock_solar_system(HELIOS_DATA_DIR "/solar_system", std::nullopt).value();
    auto simulation =
        helios::sim::Simulation::make(std::move(universe.tree), std::move(universe.catalog),
                                      helios::app::epoch_from_unix_utc(1'790'000'000.0).value(), {})
            .value();
    constexpr double k_astronomical_unit_m = 1.495978707e11;
    const auto sun = simulation.catalog().find("Sun").value();
    const auto probe = simulation
                           .add_vessel("probe",
                                       {.position_m = {0.0, -160.0 * k_astronomical_unit_m, 0.0},
                                        .velocity_m_s = {1'000.0, -16'000.0, 5'000.0}},
                                       sun)
                           .value();
    simulation.time_warp().request_level(helios::sim::k_warp_levels.size() - 1);
    for (int frame = 0; frame < 120; ++frame) {
        ASSERT_TRUE(simulation.advance(1.0 / 60.0).has_value());
        ASSERT_EQ(simulation.effective_warp(), 1e9) << frame;
    }
    const helios::sim::Vessel& vessel = simulation.vessels()[probe.index];
    EXPECT_EQ(vessel.status, helios::sim::VesselStatus::Flying);
    EXPECT_EQ(vessel.state.domain, sun);
    EXPECT_GT(helios::math::norm(vessel.state.state_in_domain.position_m), 350.0 * k_astronomical_unit_m);
    EXPECT_LT(vessel.propagator.statistics().accepted_steps, 20'000U);
}

// The demo's translunar injection must actually reach the Moon on the first pass, for any start
// date: the burn is timed from where the Moon will be when the transfer arrives.
TEST(DemoScenario, LunarProbeEntersTheMoonsSphereOfInfluenceOnItsFirstApproach) {
    for (const double start_unix_s : {1'812'345'678.0}) {
        helios::sim::Simulation simulation = make_demo(start_unix_s);
        const auto moon = simulation.catalog().find("Moon").value();
        ASSERT_EQ(simulation.vessels().size(), 3U);
        const Epoch start = simulation.now();
        std::optional<double> entered_s;
        constexpr double k_step_s = 600.0;
        for (int step = 1; step <= static_cast<int>(8.0 * k_day_s / k_step_s) && !entered_s.has_value();
             ++step) {
            ASSERT_TRUE(simulation.advance_to(start.advanced_by(k_step_s * step).value()).has_value());
            if (simulation.vessels()[1].state.domain == moon) {
                entered_s = k_step_s * step;
            }
        }
        ASSERT_TRUE(entered_s.has_value()) << start_unix_s;
        EXPECT_GT(*entered_s, 3.0 * k_day_s) << start_unix_s;
        EXPECT_LT(*entered_s, 6.0 * k_day_s) << start_unix_s;
        EXPECT_EQ(simulation.vessels()[1].status, helios::sim::VesselStatus::Flying);
        EXPECT_EQ(simulation.vessels()[0].status, helios::sim::VesselStatus::Flying);
    }
}

// The Phase 3 demo: vessels built from the stock parts, flown through the simulation host the
// way the client does, by posting commands and reading snapshots.
TEST(DemoScenario, TheDemoCraftFlyFromTheirControlBus) {
    helios::sim::Simulation simulation = make_demo(1'790'000'000.0);
    const auto added = helios::app::add_demo_craft(simulation, HELIOS_DATA_DIR);
    ASSERT_TRUE(added.has_value()) << helios::core::describe(added.error());
    ASSERT_EQ(simulation.vessels().size(), 5U);
    EXPECT_EQ(simulation.vessels()[3].name, "Kestrel");
    EXPECT_EQ(simulation.vessels()[4].name, "Firefly");
    EXPECT_EQ(helios::app::add_demo_craft(simulation, "no/such/data").error().code,
              helios::core::ErrorCode::FileNotFound);

    const helios::sim::VesselId kestrel{3};
    helios::app::SimulationHost host(std::move(simulation), helios::sim::Focus::vessel(kestrel));
    const auto signal = [&](std::string_view name) {
        const auto snapshot = host.latest();
        const auto& signals = snapshot->vessels[kestrel.index].signals;
        const auto match = std::ranges::find(signals, name, &helios::sim::SignalView::name);
        EXPECT_NE(match, signals.end()) << name;
        return match == signals.end() ? helios::sim::SignalView{} : *match;
    };
    EXPECT_TRUE(signal("engine/throttle").command);
    EXPECT_FALSE(signal("engine/throttle").toggle);
    EXPECT_TRUE(signal("engine/ignition").toggle);
    EXPECT_EQ(signal("staging/stage").maximum, 2.0);
    EXPECT_FALSE(signal("vessel/mass_kg").command);
    EXPECT_EQ(signal("vessel/mass_kg").unit, "kg");
    EXPECT_TRUE(host.latest()->vessels[0].signals.empty()); // the station has no parts

    for (const auto& [name, value] : {std::pair{"engine/throttle", 1.0}, std::pair{"staging/stage", 1.0}}) {
        host.post([kestrel, name, value](helios::sim::Simulation& posted_to) {
            EXPECT_TRUE(
                posted_to.command(kestrel, name, helios::vessel::ControlSource::Pilot, value).has_value());
        });
    }
    for (int tick = 0; tick < 120; ++tick) {
        host.step(1.0 / 60.0);
    }
    EXPECT_TRUE(host.latest()->vessels[kestrel.index].thrusting);
    EXPECT_DOUBLE_EQ(signal("vessel/thrust_n").value, 300'000.0);
    EXPECT_NEAR(signal("vessel/mass_kg").value, 25'490.0 - 98.0 * 2.0, 1e-6);
    EXPECT_EQ(signal("staging/stage").value, 1.0);
}

TEST(SimulationHost, CommandsRunBeforeTheTickAndSnapshotsFollowTheFocus) {
    helios::app::SimulationHost host(make_demo(1'790'000'000.0),
                                     helios::sim::Focus::body(helios::bodies::BodyId{3}));
    ASSERT_NE(host.latest(), nullptr);
    host.post([](helios::sim::Simulation& simulation) { simulation.time_warp().request_level(5); });
    host.set_focus(helios::sim::Focus::vessel(helios::sim::VesselId{0}));
    host.step(0.5);
    const auto snapshot = host.latest();
    EXPECT_EQ(snapshot->focus_name, "Station");
    EXPECT_EQ(snapshot->requested_warp_factor, 1e3);
    EXPECT_NEAR(snapshot->warp_factor, 1e3, 1e-9);
}

} // namespace
