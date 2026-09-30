#include "helios/sim/solar_system.hpp"

#include <gtest/gtest.h>
#include <optional>

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

// The demo's translunar injection must actually reach the Moon on the first pass, for any start
// date: the burn is timed from where the Moon will be when the transfer arrives.
TEST(DemoScenario, LunarProbeEntersTheMoonsSphereOfInfluenceOnItsFirstApproach) {
    for (const double start_unix_s : {1'812'345'678.0}) {
        helios::sim::Simulation simulation = make_demo(start_unix_s);
        const auto moon = simulation.catalog().find("Moon").value();
        ASSERT_EQ(simulation.vessels().size(), 2U);
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
