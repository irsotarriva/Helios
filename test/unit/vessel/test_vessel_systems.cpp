#include "helios/vessel/vessel_systems.hpp"

#include <gtest/gtest.h>
#include <optional>
#include <string_view>
#include <vector>

#include "support.hpp"

namespace {

using helios::core::ErrorCode;
using helios::test::after;
using helios::test::load_stock_parts;
using helios::test::make_demo_vessel;
using helios::time::Epoch;
using helios::vessel::ControlSource;
using helios::vessel::PropulsionState;
using helios::vessel::Separation;
using helios::vessel::VesselSystems;

constexpr double k_kestrel_mass_kg = 250.0 + 4400.0 + 180.0 + 60.0 + 19'500.0 + 1100.0;
constexpr double k_upper_stage_mass_kg = 250.0 + 4400.0 + 180.0;
constexpr double k_booster_burn_s = 18'000.0 / 98.0;

struct VesselSystemsTest : ::testing::Test {
    void command(VesselSystems& systems, std::string_view signal, double value, const Epoch& instant) {
        const auto commanded = systems.command(signal, ControlSource::Pilot, value, instant, separations);
        ASSERT_TRUE(commanded.has_value()) << helios::core::describe(commanded.error());
    }

    [[nodiscard]] static double reading(VesselSystems& systems, std::string_view signal,
                                        const Epoch& instant) {
        EXPECT_TRUE(systems.report_telemetry(instant).has_value());
        return systems.bus().value(systems.bus().find(signal).value());
    }

    helios::vessel::PartCatalog catalog = load_stock_parts();
    Epoch start = Epoch::from_seconds(1000.0).value();
    std::vector<Separation> separations;
};

TEST_F(VesselSystemsTest, AVesselExposesItsPartsAndInterfacesAsSignals) {
    VesselSystems kestrel = make_demo_vessel(catalog, "Kestrel", start);
    EXPECT_DOUBLE_EQ(kestrel.mass_kg(start), k_kestrel_mass_kg);
    for (const char* signal :
         {"staging/stage", "guidance/frame", "engine/throttle", "engine/ignition", "separator/separate",
          "upper_engine/throttle", "lower_engine/ignition", "decoupler/separate", "lower_tank/lox_kg",
          "upper_engine/chamber_temperature_k", "vessel/mass_kg"}) {
        EXPECT_TRUE(kestrel.bus().find(signal).has_value()) << signal;
    }
    EXPECT_DOUBLE_EQ(reading(kestrel, "vessel/mass_kg", start), k_kestrel_mass_kg);
    EXPECT_DOUBLE_EQ(reading(kestrel, "lower_tank/lox_kg", start), 12'600.0);
    EXPECT_EQ(kestrel.propulsion().thrust_n, 0.0);
    EXPECT_FALSE(kestrel.next_change().has_value());
}

TEST_F(VesselSystemsTest, AnEngineBurnsItsPropellantsAndStopsWhenTheyRunOut) {
    VesselSystems kestrel = make_demo_vessel(catalog, "Kestrel", start);
    command(kestrel, "engine/throttle", 1.0, start);
    EXPECT_EQ(kestrel.propulsion().thrust_n, 0.0); // not ignited
    command(kestrel, "staging/stage", 1.0, start);
    EXPECT_DOUBLE_EQ(kestrel.propulsion().thrust_n, 300'000.0);
    EXPECT_DOUBLE_EQ(kestrel.propulsion().mass_flow_kg_s, 98.0);
    EXPECT_DOUBLE_EQ(kestrel.propulsion().mass_kg, k_kestrel_mass_kg);

    const Epoch midway = after(start, 100.0);
    EXPECT_DOUBLE_EQ(kestrel.mass_kg(midway), k_kestrel_mass_kg - 9800.0);
    EXPECT_DOUBLE_EQ(reading(kestrel, "lower_tank/lox_kg", midway), 12'600.0 - 6860.0);
    EXPECT_DOUBLE_EQ(reading(kestrel, "lower_engine/thrust_n", midway), 300'000.0);
    EXPECT_DOUBLE_EQ(reading(kestrel, "lower_engine/chamber_temperature_k", midway), 3650.0);
    EXPECT_DOUBLE_EQ(reading(kestrel, "upper_engine/thrust_n", midway), 0.0);

    // Both propellants run out together, at an instant known in advance.
    ASSERT_TRUE(kestrel.next_change().has_value());
    EXPECT_NEAR(helios::time::seconds_between(start, *kestrel.next_change()), k_booster_burn_s, 1e-9);
    ASSERT_TRUE(kestrel.advance_to(after(start, 500.0), separations).has_value());
    EXPECT_EQ(kestrel.propulsion().thrust_n, 0.0);
    EXPECT_EQ(kestrel.propulsion().mass_flow_kg_s, 0.0);
    EXPECT_DOUBLE_EQ(kestrel.mass_kg(after(start, 500.0)), k_kestrel_mass_kg - 18'000.0);
    EXPECT_EQ(reading(kestrel, "lower_tank/lox_kg", after(start, 500.0)), 0.0);
    // The decoupler has no crossfeed: the upper tank is untouched.
    EXPECT_EQ(reading(kestrel, "upper_tank/lox_kg", after(start, 500.0)), 2800.0);
    EXPECT_FALSE(kestrel.next_change().has_value());
}

TEST_F(VesselSystemsTest, StoresAreAClosedFormOfTimeWhateverTheCadence) {
    VesselSystems coarse = make_demo_vessel(catalog, "Kestrel", start);
    command(coarse, "engine/throttle", 0.8, start);
    command(coarse, "staging/stage", 1.0, start);
    VesselSystems fine = coarse;
    const Epoch end = after(start, 150.0);
    for (int step = 1; step <= 1500; ++step) {
        ASSERT_TRUE(fine.advance_to(after(start, 0.1 * step), separations).has_value());
    }
    ASSERT_TRUE(coarse.advance_to(end, separations).has_value());
    EXPECT_EQ(fine.mass_kg(end), coarse.mass_kg(end));
    EXPECT_EQ(fine.next_change(), coarse.next_change());
    EXPECT_EQ(fine.propulsion().epoch, coarse.propulsion().epoch);
}

TEST_F(VesselSystemsTest, ARepeatedCommandDoesNotRestartTheSegment) {
    VesselSystems kestrel = make_demo_vessel(catalog, "Kestrel", start);
    command(kestrel, "engine/throttle", 1.0, start);
    command(kestrel, "staging/stage", 1.0, start);
    const std::optional<Epoch> burnout = kestrel.next_change();
    command(kestrel, "engine/throttle", 1.0, after(start, 37.123));
    EXPECT_EQ(kestrel.propulsion().epoch, start);
    EXPECT_EQ(kestrel.next_change(), burnout);
    // A real change does.
    command(kestrel, "engine/throttle", 0.9, after(start, 40.0));
    EXPECT_EQ(kestrel.propulsion().epoch, after(start, 40.0));
    EXPECT_DOUBLE_EQ(kestrel.propulsion().thrust_n, 270'000.0);
    EXPECT_DOUBLE_EQ(kestrel.propulsion().mass_kg, k_kestrel_mass_kg - 40.0 * 98.0);
}

TEST_F(VesselSystemsTest, ARunningEngineNeverGoesBelowItsMinimumLevel) {
    VesselSystems kestrel = make_demo_vessel(catalog, "Kestrel", start);
    command(kestrel, "staging/stage", 1.0, start);
    command(kestrel, "engine/throttle", 0.1, start);
    EXPECT_DOUBLE_EQ(kestrel.propulsion().thrust_n, 0.6 * 300'000.0);
    command(kestrel, "engine/throttle", 0.0, start);
    EXPECT_EQ(kestrel.propulsion().thrust_n, 0.0);
    // The engine's own signal overrides the one all engines follow.
    command(kestrel, "engine/throttle", 1.0, start);
    command(kestrel, "lower_engine/throttle", 0.0, start);
    EXPECT_EQ(kestrel.propulsion().thrust_n, 0.0);
}

TEST_F(VesselSystemsTest, StagingDropsTheBoosterAndLightsTheUpperEngine) {
    VesselSystems kestrel = make_demo_vessel(catalog, "Kestrel", start);
    command(kestrel, "engine/throttle", 1.0, start);
    command(kestrel, "staging/stage", 1.0, start);
    const Epoch staging = after(start, 200.0); // after the booster has burnt out
    command(kestrel, "staging/stage", 2.0, staging);

    ASSERT_EQ(separations.size(), 1U);
    const Separation& separation = separations.front();
    EXPECT_EQ(separation.epoch, staging);
    EXPECT_EQ(kestrel.assembly().parts().size(), 3U);
    EXPECT_DOUBLE_EQ(kestrel.mass_kg(staging), k_upper_stage_mass_kg);
    EXPECT_EQ(separation.systems.assembly().parts().size(), 3U);
    EXPECT_EQ(separation.systems.assembly().parts().front().name, "decoupler");
    EXPECT_DOUBLE_EQ(separation.systems.mass_kg(staging), 60.0 + 1500.0 + 1100.0);
    // The upper stage is ahead and is pushed forwards; momentum is conserved.
    EXPECT_GT(separation.kept_delta_v_m_s, 0.0);
    EXPECT_NEAR(separation.kept_delta_v_m_s * k_upper_stage_mass_kg
                    + separation.separated_delta_v_m_s * separation.systems.mass_kg(staging),
                0.0, 1e-9);
    EXPECT_DOUBLE_EQ(separation.kept_delta_v_m_s, 2500.0 / k_upper_stage_mass_kg);

    EXPECT_DOUBLE_EQ(kestrel.propulsion().thrust_n, 30'000.0);
    EXPECT_DOUBLE_EQ(kestrel.propulsion().mass_flow_kg_s, 9.0);
    EXPECT_FALSE(kestrel.bus().find("lower_engine/ignition").has_value());
    EXPECT_TRUE(separation.systems.bus().find("lower_engine/ignition").has_value());
    // The booster's engine is still commanded on, but has nothing left to burn.
    EXPECT_EQ(separation.systems.propulsion().thrust_n, 0.0);
}

TEST_F(VesselSystemsTest, AnEngineStarvesWithoutPowerAndRunsOnWhatThePanelsGive) {
    VesselSystems firefly = make_demo_vessel(catalog, "Firefly", start);
    command(firefly, "engine/throttle", 1.0, start);
    command(firefly, "staging/stage", 1.0, start);
    EXPECT_DOUBLE_EQ(firefly.propulsion().thrust_n, 0.25);
    EXPECT_DOUBLE_EQ(firefly.propulsion().mass_flow_kg_s, 8.5e-6); // charge has no mass
    // With the panels producing more than the thruster draws, only the xenon runs out.
    EXPECT_NEAR(helios::time::seconds_between(start, *firefly.next_change()), 120.0 / 8.5e-6, 1e-3);

    // Panels folded: the battery lasts 3.6 MJ / 5 kW = 720 s.
    command(firefly, "panel_left/deployed", 0.0, start);
    command(firefly, "panel_right/deployed", 0.0, start);
    EXPECT_NEAR(helios::time::seconds_between(start, *firefly.next_change()), 720.0, 1e-9);
    const Epoch dark = after(start, 1000.0);
    ASSERT_TRUE(firefly.advance_to(dark, separations).has_value());
    EXPECT_EQ(firefly.propulsion().thrust_n, 0.0);
    EXPECT_EQ(reading(firefly, "probe/charge_fraction", dark), 0.0);
    EXPECT_DOUBLE_EQ(reading(firefly, "xenon_tank/xenon_kg", dark), 120.0 - 720.0 * 8.5e-6);

    // One panel gives 6 kW: enough to run the thruster on an empty battery, and to recharge it
    // with the 1 kW to spare.
    command(firefly, "panel_left/deployed", 1.0, dark);
    EXPECT_DOUBLE_EQ(firefly.propulsion().thrust_n, 0.25);
    EXPECT_NEAR(helios::time::seconds_between(dark, *firefly.next_change()), 3600.0, 1e-9);
    EXPECT_NEAR(reading(firefly, "probe/charge_fraction", after(dark, 1800.0)), 0.5, 1e-12);
}

TEST_F(VesselSystemsTest, TheForecastMatchesWhatThenHappens) {
    VesselSystems kestrel = make_demo_vessel(catalog, "Kestrel", start);
    command(kestrel, "engine/throttle", 1.0, start);
    const Epoch ignition = after(start, 60.0);
    const Epoch staging = after(start, 300.0);
    const Epoch cutoff = after(start, 400.0);
    // The throttle was opened by the pilot, so only the pilot can close it.
    ASSERT_TRUE(
        kestrel
            .schedule(
                {.epoch = cutoff, .signal = "engine/throttle", .source = ControlSource::Pilot, .value = 0.0})
            .has_value());
    ASSERT_TRUE(kestrel.schedule({.epoch = ignition, .signal = "staging/stage", .value = 1.0}).has_value());
    ASSERT_TRUE(kestrel.schedule({.epoch = staging, .signal = "staging/stage", .value = 2.0}).has_value());
    EXPECT_EQ(kestrel.program().front().epoch, ignition); // kept in time order

    const auto forecast = kestrel.forecast(16);
    ASSERT_TRUE(forecast.has_value());
    // Coast, booster burn, burn-out, upper stage burn (after the separation), cut-off.
    ASSERT_EQ(forecast->size(), 5U);
    EXPECT_EQ((*forecast)[0].epoch, start);
    EXPECT_EQ((*forecast)[0].thrust_n, 0.0);
    EXPECT_EQ((*forecast)[1].epoch, ignition);
    EXPECT_DOUBLE_EQ((*forecast)[1].thrust_n, 300'000.0);
    EXPECT_NEAR(helios::time::seconds_between(ignition, (*forecast)[2].epoch), k_booster_burn_s, 1e-9);
    EXPECT_EQ((*forecast)[2].thrust_n, 0.0);
    EXPECT_EQ((*forecast)[3].epoch, staging);
    EXPECT_DOUBLE_EQ((*forecast)[3].thrust_n, 30'000.0);
    EXPECT_DOUBLE_EQ((*forecast)[3].mass_kg, k_upper_stage_mass_kg);
    EXPECT_EQ((*forecast)[4].epoch, cutoff);
    EXPECT_EQ((*forecast)[4].thrust_n, 0.0);
    EXPECT_DOUBLE_EQ((*forecast)[4].mass_kg, k_upper_stage_mass_kg - 900.0);
    // Forecasting did not touch the vessel.
    EXPECT_EQ(kestrel.assembly().parts().size(), 6U);
    EXPECT_EQ(kestrel.program().size(), 3U);

    // Living through it, at an arbitrary cadence, gives exactly the forecast states.
    std::vector<PropulsionState> lived{kestrel.propulsion()};
    for (int step = 1; step <= 450; ++step) {
        ASSERT_TRUE(kestrel.advance_to(after(start, 1.37 * step), separations).has_value());
        if (kestrel.propulsion().epoch != lived.back().epoch) {
            lived.push_back(kestrel.propulsion());
        }
    }
    ASSERT_EQ(lived.size(), forecast->size());
    for (std::size_t index = 0; index < lived.size(); ++index) {
        EXPECT_EQ(lived[index].epoch, (*forecast)[index].epoch) << index;
        EXPECT_EQ(lived[index].thrust_n, (*forecast)[index].thrust_n) << index;
        EXPECT_EQ(lived[index].mass_flow_kg_s, (*forecast)[index].mass_flow_kg_s) << index;
        EXPECT_EQ(lived[index].mass_kg, (*forecast)[index].mass_kg) << index;
    }
    EXPECT_EQ(separations.size(), 1U);
    EXPECT_TRUE(kestrel.program().empty());
    // A forecast can be cut short.
    EXPECT_EQ(make_demo_vessel(catalog, "Kestrel", start).forecast(1).value().size(), 1U);
}

TEST_F(VesselSystemsTest, PointingFollowsTheGuidanceSignals) {
    VesselSystems kestrel = make_demo_vessel(catalog, "Kestrel", start);
    using helios::vessel::Pointing;
    EXPECT_EQ(kestrel.propulsion().pointing, (Pointing{Pointing::Frame::Orbital, {1.0, 0.0, 0.0}}));
    command(kestrel, "guidance/x", -1.0, start);
    EXPECT_EQ(kestrel.propulsion().pointing.direction.x, -1.0); // retrograde
    command(kestrel, "guidance/frame", 0.0, start);
    command(kestrel, "guidance/x", 0.0, start);
    command(kestrel, "guidance/y", 0.6, start);
    command(kestrel, "guidance/z", 0.8, start);
    EXPECT_EQ(kestrel.propulsion().pointing.frame, Pointing::Frame::Inertial);
    EXPECT_DOUBLE_EQ(kestrel.propulsion().pointing.direction.z, 0.8);
}

TEST_F(VesselSystemsTest, RejectsInvalidCommands) {
    VesselSystems kestrel = make_demo_vessel(catalog, "Kestrel", start);
    EXPECT_EQ(
        kestrel.command("warp_core/throttle", ControlSource::Pilot, 1.0, start, separations).error().code,
        ErrorCode::InvalidArgument);
    EXPECT_EQ(kestrel.command("vessel/mass_kg", ControlSource::Pilot, 1.0, start, separations).error().code,
              ErrorCode::InvalidArgument);
    command(kestrel, "engine/throttle", 0.5, after(start, 10.0));
    EXPECT_EQ(kestrel.command("engine/throttle", ControlSource::Pilot, 1.0, start, separations).error().code,
              ErrorCode::OutOfRange);
    EXPECT_EQ(kestrel.schedule({.epoch = start, .signal = "engine/throttle", .value = 1.0}).error().code,
              ErrorCode::OutOfRange);
    EXPECT_EQ(VesselSystems::make({}, {}, {}, start).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(kestrel.amount(99, 0, start), 0.0);
    // Releasing gives the signal back to its default.
    ASSERT_TRUE(kestrel.release("engine/throttle", ControlSource::Pilot, after(start, 20.0), separations)
                    .has_value());
    EXPECT_EQ(kestrel.bus().value(kestrel.bus().find("engine/throttle").value()), 0.0);
}

} // namespace
