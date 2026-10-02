#include "helios/sim/solar_system.hpp"

#include <algorithm>
#include <gtest/gtest.h>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include "flight_controls.hpp"
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
    ASSERT_EQ(simulation.vessels().size(), 10U);
    EXPECT_EQ(simulation.vessels()[3].name, "Kestrel");
    EXPECT_EQ(simulation.vessels()[4].name, "Firefly");
    EXPECT_EQ(simulation.vessels()[5].name, "Heron");
    EXPECT_EQ(simulation.vessels()[5].status, helios::sim::VesselStatus::Landed);
    EXPECT_EQ(simulation.vessels()[5].state.domain, simulation.catalog().find("Moon").value());
    EXPECT_EQ(simulation.vessels()[6].name, "Merlin");
    EXPECT_EQ(simulation.vessels()[7].name, "Osprey");
    EXPECT_EQ(simulation.vessels()[7].status, helios::sim::VesselStatus::Landed);
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
    // The focused vessel is the one being flown: it is in the physics bubble, the others are not.
    EXPECT_TRUE(host.latest()->vessels[kestrel.index].in_bubble);
    EXPECT_FALSE(host.latest()->vessels[4].in_bubble);
    EXPECT_DOUBLE_EQ(signal("vessel/thrust_n").value, 300'000.0);
    EXPECT_NEAR(signal("vessel/mass_kg").value, 25'490.0 - 98.0 * 2.0, 1e-6);
    EXPECT_EQ(signal("staging/stage").value, 1.0);
}

// The Phase 4 demo: the lander is flown with nothing but the instruments of its cockpit, the
// way the client does it when the pilot clicks and drags them.
TEST(DemoScenario, OspreyIsFlownFromItsCockpit) {
    helios::sim::Simulation simulation = make_demo(1'790'000'000.0);
    ASSERT_TRUE(helios::app::add_demo_craft(simulation, HELIOS_DATA_DIR).has_value());
    const helios::sim::VesselId osprey{7};
    helios::app::SimulationHost host(std::move(simulation), helios::sim::Focus::vessel(osprey));
    const auto vessel = [&] { return host.latest()->vessels[osprey.index]; };
    const auto reading = [&](std::string_view name) {
        const helios::sim::VesselView view = vessel();
        const auto match = std::ranges::find(view.signals, name, &helios::sim::SignalView::name);
        EXPECT_NE(match, view.signals.end()) << name;
        return match == view.signals.end() ? 0.0 : match->value;
    };
    const auto post = [&](const std::vector<helios::app::SignalCommand>& commands) {
        for (const helios::app::SignalCommand& command : commands) {
            host.post([posted = command](helios::sim::Simulation& posted_to) {
                EXPECT_TRUE(posted_to
                                .command(posted.vessel, posted.signal, helios::vessel::ControlSource::Pilot,
                                         posted.value)
                                .has_value());
            });
        }
    };
    const auto fly = [&](double seconds) {
        for (int tick = 0; tick < static_cast<int>(seconds * 60.0); ++tick) {
            host.step(1.0 / 60.0);
        }
    };

    // The cabin is the part with the seat; its instruments are found by what is written on them.
    const helios::sim::VesselView parked = vessel();
    ASSERT_EQ(parked.status, helios::sim::VesselStatus::Landed);
    const auto cabin = std::ranges::find_if(
        parked.parts, [](const helios::sim::PartView& part) { return part.datasheet->cockpit.has_value(); });
    ASSERT_NE(cabin, parked.parts.end());
    const std::vector<helios::vessel::Instrument> instruments = cabin->datasheet->cockpit->instruments;
    const auto labelled = [&](std::string_view label) {
        const auto match = std::ranges::find(instruments, label, &helios::vessel::Instrument::label);
        EXPECT_NE(match, instruments.end()) << label;
        return match == instruments.end() ? helios::vessel::Instrument{} : *match;
    };
    // Every instrument of the panel is bound to a signal this vessel has.
    for (const helios::vessel::Instrument& instrument : instruments) {
        if (!instrument.signal.empty()) {
            EXPECT_NE(std::ranges::find(parked.signals, instrument.signal, &helios::sim::SignalView::name),
                      parked.signals.end())
                << instrument.label;
        }
    }
    const double standing_m = reading("nav/altitude_m");
    EXPECT_NEAR(standing_m, 3.0, 0.5);

    // STAGE arms the engine, which shows on the IGN switch; the lever opens the throttle.
    post(helios::app::press_instrument(parked, labelled("STAGE")));
    post({helios::app::move_lever(parked, labelled("THROTTLE"), 0.9).value()});
    fly(5.0);
    EXPECT_EQ(vessel().status, helios::sim::VesselStatus::Flying);
    EXPECT_TRUE(vessel().in_bubble);
    EXPECT_EQ(reading("engine/ignition"), 1.0);
    EXPECT_DOUBLE_EQ(reading("vessel/thrust_n"), 0.9 * 16'000.0);
    EXPECT_GT(reading("nav/altitude_m"), standing_m + 5.0);
    EXPECT_GT(reading("nav/vertical_speed_m_s"), 2.0);

    // The IGN switch, thrown by hand, shuts the engine down though the sequencer lit it.
    post(helios::app::press_instrument(vessel(), labelled("IGN")));
    fly(0.5);
    EXPECT_EQ(reading("engine/ignition"), 0.0);
    EXPECT_EQ(reading("vessel/thrust_n"), 0.0);
}

// The pilot goes with the focus: into the seat of a vessel that has one, and out of it into the
// cabin where there is one.
TEST(DemoScenario, ThePilotBoardsTheFocusedVesselAndWalksAboutPetrel) {
    helios::sim::Simulation simulation = make_demo(1'790'000'000.0);
    ASSERT_TRUE(helios::app::add_demo_craft(simulation, HELIOS_DATA_DIR).has_value());
    ASSERT_EQ(simulation.vessels()[8].name, "Petrel");
    ASSERT_EQ(simulation.vessels()[9].name, "Albatross");
    const helios::sim::VesselId heron{5};
    const helios::sim::VesselId petrel{8};
    helios::app::SimulationHost host(std::move(simulation), helios::sim::Focus::vessel(heron));
    EXPECT_FALSE(host.latest()->pilot.has_value()); // Heron has no seat
    host.set_focus(helios::sim::Focus::vessel(petrel));
    host.step(1.0 / 60.0);
    ASSERT_TRUE(host.latest()->pilot.has_value());
    EXPECT_EQ(host.latest()->pilot->vessel, petrel);
    EXPECT_TRUE(host.latest()->pilot->seated);

    host.post([](helios::sim::Simulation& posted_to) { EXPECT_TRUE(posted_to.leave_seat().has_value()); });
    for (int tick = 0; tick < 120; ++tick) {
        host.step(1.0 / 60.0);
    }
    const helios::sim::PilotView pilot = host.latest()->pilot.value();
    EXPECT_FALSE(pilot.seated);
    EXPECT_TRUE(pilot.standing);
    EXPECT_NEAR(pilot.up.x, 1.0, 1e-3); // head towards the nose, which points at the sky
    // Looking at a body does not take the pilot out of the vessel.
    host.set_focus(helios::sim::Focus::body(helios::bodies::BodyId{3}));
    host.step(1.0 / 60.0);
    EXPECT_EQ(host.latest()->pilot->vessel, petrel);
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
