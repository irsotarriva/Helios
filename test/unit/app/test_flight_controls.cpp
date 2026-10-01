#include <algorithm>
#include <gtest/gtest.h>
#include <string_view>
#include <vector>

#include "flight_controls.hpp"

namespace {

using helios::app::Action;
using helios::app::ActionState;
using helios::app::flight_commands;
using helios::app::FlightControlState;
using helios::app::SignalCommand;
using helios::math::Matrix3;
using helios::sim::VesselView;
using helios::vessel::Instrument;

// A vessel with an engine, reaction wheels and one stage left.
[[nodiscard]] VesselView make_vessel(double throttle) {
    VesselView vessel;
    vessel.id = {3};
    const auto command = [&](std::string_view name, double value, double minimum, double maximum,
                             bool toggle = false) {
        vessel.signals.push_back({.name = std::string{name},
                                  .command = true,
                                  .toggle = toggle,
                                  .value = value,
                                  .minimum = minimum,
                                  .maximum = maximum});
    };
    command("engine/throttle", throttle, 0.0, 1.0);
    command("attitude/roll", 0.0, -1.0, 1.0);
    command("attitude/pitch", 0.0, -1.0, 1.0);
    command("attitude/yaw", 0.0, -1.0, 1.0);
    command("staging/stage", 0.0, 0.0, 1.0);
    command("guidance/hold", 1.0, 0.0, 1.0, true);
    command("guidance/frame", 1.0, 0.0, 2.0);
    vessel.signals.push_back({.name = "vessel/mass_kg", .unit = "kg", .value = 5000.0});
    return vessel;
}

[[nodiscard]] ActionState holding(std::initializer_list<Action> actions, bool pressed = false) {
    ActionState state;
    for (const Action action : actions) {
        state.set(action, true, pressed);
    }
    return state;
}

[[nodiscard]] const SignalCommand& find(const std::vector<SignalCommand>& commands, std::string_view signal) {
    const auto match = std::ranges::find(commands, signal, &SignalCommand::signal);
    EXPECT_NE(match, commands.end()) << signal;
    return match == commands.end() ? commands.front() : *match;
}

TEST(FlightControls, TheThrottleKeysRampAndTheOthersJump) {
    const VesselView vessel = make_vessel(0.25);
    FlightControlState state;
    // Half the range per second: 0.25 s adds an eighth, starting from where the lever is.
    auto commands = flight_commands(vessel, holding({Action::ThrottleUp}), Matrix3{}, 0.25, state);
    ASSERT_EQ(commands.size(), 1U);
    EXPECT_EQ(commands[0].vessel, vessel.id);
    EXPECT_EQ(commands[0].signal, "engine/throttle");
    EXPECT_DOUBLE_EQ(commands[0].value, 0.375);
    // The snapshot has not caught up; the key goes on from its own value, and stops at the top.
    commands = flight_commands(vessel, holding({Action::ThrottleUp}), Matrix3{}, 0.25, state);
    EXPECT_DOUBLE_EQ(commands[0].value, 0.5);
    commands = flight_commands(vessel, holding({Action::ThrottleUp}), Matrix3{}, 10.0, state);
    EXPECT_DOUBLE_EQ(commands[0].value, 1.0);
    // Released: nothing is commanded, and the next press starts from the lever again.
    EXPECT_TRUE(flight_commands(vessel, ActionState{}, Matrix3{}, 0.25, state).empty());
    commands = flight_commands(vessel, holding({Action::ThrottleDown}), Matrix3{}, 0.25, state);
    EXPECT_DOUBLE_EQ(commands[0].value, 0.125);

    EXPECT_DOUBLE_EQ(
        flight_commands(vessel, holding({Action::ThrottleFull}, true), Matrix3{}, 0.01, state)[0].value, 1.0);
    EXPECT_DOUBLE_EQ(
        flight_commands(vessel, holding({Action::ThrottleCut}, true), Matrix3{}, 0.01, state)[0].value, 0.0);
}

TEST(FlightControls, SteeringIsAskedInTheSeatsAxesAndReleasedWithTheKey) {
    const VesselView vessel = make_vessel(0.0);
    FlightControlState state;
    // Seat axes are the vessel's: pulling the nose up is a turn about −y.
    auto commands = flight_commands(vessel, holding({Action::PitchUp}), Matrix3{}, 0.02, state);
    ASSERT_EQ(commands.size(), 3U);
    EXPECT_EQ(find(commands, "attitude/pitch").value, -1.0);
    EXPECT_EQ(find(commands, "attitude/roll").value, 0.0);
    EXPECT_EQ(find(commands, "attitude/yaw").value, 0.0);
    EXPECT_FALSE(commands[0].release);
    EXPECT_TRUE(state.steering);
    // Key up: the pilot lets go of all three, once, so that the attitude hold has them again.
    commands = flight_commands(vessel, ActionState{}, Matrix3{}, 0.02, state);
    ASSERT_EQ(commands.size(), 3U);
    EXPECT_TRUE(std::ranges::all_of(commands, &SignalCommand::release));
    EXPECT_TRUE(flight_commands(vessel, ActionState{}, Matrix3{}, 0.02, state).empty());

    // A seat that faces the vessel's +z with the nose overhead (a lander): yawing left, about
    // the pilot's up, is a turn about the vessel's nose.
    const Matrix3 sideways{{0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0}};
    commands = flight_commands(vessel, holding({Action::YawLeft}), sideways, 0.02, state);
    EXPECT_EQ(find(commands, "attitude/roll").value, 1.0);
    EXPECT_EQ(find(commands, "attitude/yaw").value, 0.0);
    // Rolling right, about where the pilot faces, is about the vessel's +z.
    commands = flight_commands(vessel, holding({Action::RollRight}), sideways, 0.02, state);
    EXPECT_EQ(find(commands, "attitude/yaw").value, 1.0);
    // Opposite keys cancel: that is no steering, and the pilot lets go.
    commands = flight_commands(vessel, holding({Action::RollRight, Action::RollLeft}), sideways, 0.02, state);
    EXPECT_TRUE(std::ranges::all_of(commands, &SignalCommand::release));
}

TEST(FlightControls, StagingAndTheHoldSwitchActOncePerPress) {
    VesselView vessel = make_vessel(0.0);
    FlightControlState state;
    auto commands = flight_commands(vessel, holding({Action::Stage, Action::ToggleAttitudeHold}, true),
                                    Matrix3{}, 0.02, state);
    EXPECT_EQ(find(commands, "staging/stage").value, 1.0);
    EXPECT_EQ(find(commands, "guidance/hold").value, 0.0);
    // Held, not pressed: nothing more.
    EXPECT_TRUE(
        flight_commands(vessel, holding({Action::Stage, Action::ToggleAttitudeHold}), Matrix3{}, 0.02, state)
            .empty());
    // No stage left: the key does nothing.
    vessel.signals[4].value = 1.0;
    EXPECT_TRUE(flight_commands(vessel, holding({Action::Stage}, true), Matrix3{}, 0.02, state).empty());
    // A vessel with no systems takes no commands at all.
    EXPECT_TRUE(flight_commands(VesselView{},
                                holding({Action::ThrottleUp, Action::PitchUp, Action::Stage}, true),
                                Matrix3{}, 0.02, state)
                    .empty());
}

TEST(FlightControls, InstrumentsPublishToTheirSignals) {
    const VesselView vessel = make_vessel(0.5);
    const Instrument hold{.kind = Instrument::Kind::Switch, .signal = "guidance/hold"};
    auto commands = helios::app::press_instrument(vessel, hold);
    ASSERT_EQ(commands.size(), 1U);
    EXPECT_EQ(commands[0].signal, "guidance/hold");
    EXPECT_EQ(commands[0].value, 0.0); // it was on

    const Instrument stage{.kind = Instrument::Kind::Button, .signal = "staging/stage", .step = 1.0};
    commands = helios::app::press_instrument(vessel, stage);
    ASSERT_EQ(commands.size(), 1U);
    EXPECT_EQ(commands[0].value, 1.0);

    const Instrument point{
        .kind = Instrument::Kind::Button,
        .commands = {{.signal = "guidance/frame", .value = 2.0}, {.signal = "guidance/hold", .value = 1.0}}};
    commands = helios::app::press_instrument(vessel, point);
    ASSERT_EQ(commands.size(), 2U);
    EXPECT_EQ(find(commands, "guidance/frame").value, 2.0);
    // All or nothing: one unknown signal and the button does nothing.
    Instrument broken = point;
    broken.commands.push_back({.signal = "no/such", .value = 1.0});
    EXPECT_TRUE(helios::app::press_instrument(vessel, broken).empty());
    // A reading cannot be commanded, and a dial is not pressed.
    EXPECT_TRUE(helios::app::press_instrument(
                    vessel, Instrument{.kind = Instrument::Kind::Switch, .signal = "vessel/mass_kg"})
                    .empty());
    EXPECT_TRUE(helios::app::press_instrument(
                    vessel, Instrument{.kind = Instrument::Kind::Dial, .signal = "engine/throttle"})
                    .empty());

    const Instrument throttle{.kind = Instrument::Kind::Lever, .signal = "engine/throttle"};
    EXPECT_EQ(helios::app::lever_fraction(vessel, throttle), 0.5);
    const auto moved = helios::app::move_lever(vessel, throttle, 1.7);
    ASSERT_TRUE(moved.has_value());
    EXPECT_EQ(moved->value, 1.0);
    EXPECT_FALSE(
        helios::app::move_lever(vessel, Instrument{.kind = Instrument::Kind::Lever, .signal = "x/y"}, 0.5)
            .has_value());
    EXPECT_FALSE(helios::app::lever_fraction(vessel, Instrument{.signal = "x/y"}).has_value());
}

} // namespace
