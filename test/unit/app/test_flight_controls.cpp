#include <algorithm>
#include <gtest/gtest.h>
#include <initializer_list>
#include <string_view>
#include <vector>

#include "flight_controls.hpp"

namespace {

using helios::app::Action;
using helios::app::ActionState;
using helios::app::ControlHold;
using helios::app::drag_control;
using helios::app::flight_commands;
using helios::app::FlightControlState;
using helios::app::let_go_control;
using helios::app::SignalCommand;
using helios::app::take_control;
using helios::math::Matrix3;
using helios::sim::VesselView;
using helios::vessel::Control;

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

// A suit's thruster pack: pushed along the seat's axes, like steering is turned about them.
TEST(FlightControls, TranslationIsAskedInTheSeatsAxesWhereTheVesselHasIt) {
    VesselView vessel = make_vessel(0.0);
    FlightControlState state;
    // A vessel that cannot be pushed sideways takes no such command.
    EXPECT_TRUE(flight_commands(vessel, holding({Action::TranslateForward}), Matrix3{}, 0.02, state).empty());
    state = {};
    for (const std::string_view name : {"translation/x", "translation/y", "translation/z"}) {
        vessel.signals.push_back(
            {.name = std::string{name}, .command = true, .value = 0.0, .minimum = -1.0, .maximum = 1.0});
    }
    // The suit's seat: the wearer faces the suit's +z, head towards its +x.
    const Matrix3 helmet{{0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0}};
    auto commands = flight_commands(vessel, holding({Action::TranslateForward}), helmet, 0.02, state);
    ASSERT_EQ(commands.size(), 3U);
    EXPECT_EQ(find(commands, "translation/z").value, 1.0);
    EXPECT_EQ(find(commands, "translation/x").value, 0.0);
    commands = flight_commands(vessel, holding({Action::TranslateDown}), helmet, 0.02, state);
    EXPECT_EQ(find(commands, "translation/x").value, -1.0);
    EXPECT_TRUE(state.translating);
    EXPECT_FALSE(state.steering);
    // Keys up: the thrusters are let go of, once.
    commands = flight_commands(vessel, ActionState{}, helmet, 0.02, state);
    ASSERT_EQ(commands.size(), 3U);
    EXPECT_TRUE(std::ranges::all_of(commands, &SignalCommand::release));
    EXPECT_TRUE(flight_commands(vessel, ActionState{}, helmet, 0.02, state).empty());
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

// A switch of `values` of one signal (BRIEFING D43).
[[nodiscard]] Control switch_of(std::string_view signal, std::initializer_list<double> values) {
    Control control{.id = "switch", .type = Control::Type::Switch, .signal = std::string{signal}};
    for (const double value : values) {
        control.positions.push_back({.settings = {{.signal = std::string{signal}, .value = value}}});
    }
    return control;
}

TEST(FlightControls, PushButtonsSetWhatTheySayAllOrNothing) {
    const VesselView vessel = make_vessel(0.5);
    ControlHold hold;
    const Control stage{
        .id = "stage", .type = Control::Type::Momentary, .signal = "staging/stage", .step = 1.0};
    auto commands = take_control(vessel, stage, 0.0, hold);
    ASSERT_EQ(commands.size(), 1U);
    EXPECT_EQ(commands[0].value, 1.0);

    Control point{
        .id = "point",
        .type = Control::Type::Momentary,
        .press = {{.signal = "guidance/frame", .value = 2.0}, {.signal = "guidance/hold", .value = 1.0}},
        .release = {{.signal = "guidance/frame", .value = 0.0}}};
    commands = take_control(vessel, point, 0.0, hold);
    ASSERT_EQ(commands.size(), 2U);
    EXPECT_EQ(find(commands, "guidance/frame").value, 2.0);
    commands = let_go_control(vessel, point, hold);
    ASSERT_EQ(commands.size(), 1U);
    EXPECT_EQ(commands[0].value, 0.0) << "what it sets when let go";
    // All or nothing: one unknown signal and the button does nothing.
    point.press.push_back({.signal = "no/such", .value = 1.0});
    EXPECT_TRUE(take_control(vessel, point, 0.0, hold).empty());
}

TEST(FlightControls, SwitchesMoveAPositionAtATime) {
    const VesselView vessel = make_vessel(0.5);
    ControlHold hold;
    // Two ways: a click anywhere flips it.
    auto commands = take_control(vessel, switch_of("guidance/hold", {0.0, 1.0}), -0.8, hold);
    ASSERT_EQ(commands.size(), 1U);
    EXPECT_EQ(commands[0].value, 0.0) << "it was on";
    // Three ways: the side clicked says which way (guidance/frame is at 1, the middle).
    const Control frame = switch_of("guidance/frame", {0.0, 1.0, 2.0});
    EXPECT_EQ(take_control(vessel, frame, 0.6, hold).at(0).value, 2.0);
    EXPECT_EQ(take_control(vessel, frame, -0.6, hold).at(0).value, 0.0);
    // A reading is not a command: nothing moves it.
    EXPECT_TRUE(take_control(vessel, switch_of("vessel/mass_kg", {0.0, 5000.0}), 0.5, hold).empty());

    // Locked: a click does nothing; pulled out and over (a drag of half a step or more), it moves.
    Control locked = frame;
    locked.locked = true;
    EXPECT_TRUE(take_control(vessel, locked, 0.6, hold).empty());
    EXPECT_TRUE(drag_control(vessel, locked, 0.3, 0.0, hold).empty());
    commands = drag_control(vessel, locked, 0.3, 0.0, hold);
    ASSERT_EQ(commands.size(), 1U);
    EXPECT_EQ(commands[0].value, 2.0);

    // Sprung: held in its sprung position it stays; let go, it goes back to rest.
    Control sprung = frame;
    sprung.positions[2].spring = true;
    sprung.rest = 1;
    VesselView pushed = make_vessel(0.5);
    std::ranges::find(pushed.signals, "guidance/frame", &helios::sim::SignalView::name)->value = 2.0;
    commands = let_go_control(pushed, sprung, hold);
    ASSERT_EQ(commands.size(), 1U);
    EXPECT_EQ(commands[0].value, 1.0);
    EXPECT_TRUE(let_go_control(vessel, sprung, hold).empty()) << "at rest already";
}

TEST(FlightControls, AnAxisFollowsTheHandAndSettlesInADetent) {
    const VesselView vessel = make_vessel(0.5);
    Control throttle{.id = "throttle", .type = Control::Type::Axis, .signal = "engine/throttle"};
    ControlHold hold;
    EXPECT_TRUE(take_control(vessel, throttle, 0.0, hold).empty());
    EXPECT_EQ(hold.fraction, 0.5) << "taken where it stands";
    auto commands = drag_control(vessel, throttle, 1.2, 0.0, hold);
    ASSERT_EQ(commands.size(), 1U);
    EXPECT_EQ(commands[0].value, 1.0) << "no further than its end";
    EXPECT_TRUE(let_go_control(vessel, throttle, hold).empty());

    throttle.detents = {0.8};
    (void)drag_control(vessel, throttle, -0.18, 0.0, hold); // to 0.82
    commands = let_go_control(vessel, throttle, hold);
    ASSERT_EQ(commands.size(), 1U);
    EXPECT_NEAR(commands[0].value, 0.8, 1e-12) << "into the detent nearby";
    throttle.spring_to = 0.0;
    EXPECT_EQ(let_go_control(vessel, throttle, hold).at(0).value, 0.0) << "a spring takes it back";
    // A dead axis does nothing.
    EXPECT_TRUE(
        drag_control(vessel, Control{.type = Control::Type::Axis, .signal = "x/y"}, 0.5, 0.0, hold).empty());
}

TEST(FlightControls, AStickDeflectsAndIsCentredWhenLetGo) {
    const VesselView vessel = make_vessel(0.5);
    const Control stick{
        .id = "stick", .type = Control::Type::Stick, .signals = {"attitude/roll", "attitude/pitch"}};
    ControlHold hold;
    EXPECT_TRUE(take_control(vessel, stick, 0.0, hold).empty());
    auto commands = drag_control(vessel, stick, 0.5, -2.0, hold);
    ASSERT_EQ(commands.size(), 2U);
    EXPECT_EQ(find(commands, "attitude/roll").value, 0.5);
    EXPECT_EQ(find(commands, "attitude/pitch").value, -1.0) << "no further than full";
    commands = let_go_control(vessel, stick, hold);
    ASSERT_EQ(commands.size(), 2U);
    EXPECT_TRUE(std::ranges::all_of(commands, &SignalCommand::release)) << "the keys and the autopilot again";
}

TEST(FlightControls, AnEncoderClicksAndMayComeRound) {
    VesselView vessel = make_vessel(0.5);
    Control knob{.id = "knob", .type = Control::Type::Encoder, .signal = "guidance/frame", .step = 1.0};
    ControlHold hold;
    EXPECT_EQ(take_control(vessel, knob, 0.4, hold).at(0).value, 2.0);
    EXPECT_EQ(take_control(vessel, knob, -0.4, hold).at(0).value, 0.0);
    std::ranges::find(vessel.signals, "guidance/frame", &helios::sim::SignalView::name)->value = 2.0;
    EXPECT_EQ(take_control(vessel, knob, 0.4, hold).at(0).value, 2.0) << "stops at its end";
    knob.wrap = true;
    knob.step = 1.5;
    EXPECT_EQ(take_control(vessel, knob, 0.4, hold).at(0).value, 1.5) << "comes round: 3.5 is 1.5";
    // Dragged: one click for each whole travel.
    EXPECT_EQ(drag_control(vessel, knob, 1.1, 0.0, hold).size(), 1U);
}

} // namespace
