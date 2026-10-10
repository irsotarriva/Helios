#include "flight_controls.hpp"

#include "helios/math/vector3.hpp"
#include "helios/vessel/vessel_systems.hpp"

#include <algorithm>
#include <functional>
#include <string_view>

namespace helios::app {

namespace {

constexpr std::string_view k_throttle = "engine/throttle";
// In the order of the vessel's axes: about x (the nose), y and z.
constexpr std::array<std::string_view, 3> k_attitude{"attitude/roll", "attitude/pitch", "attitude/yaw"};
constexpr std::array<std::string_view, 3> k_translation{"translation/x", "translation/y", "translation/z"};

[[nodiscard]] std::optional<std::reference_wrapper<const sim::SignalView>>
find_signal(const sim::VesselView& vessel, std::string_view name) {
    const auto match = std::ranges::find(vessel.signals, name, &sim::SignalView::name);
    if (match == vessel.signals.end() || !match->command) {
        return std::nullopt;
    }
    return std::cref(*match);
}

[[nodiscard]] double axis(const ActionState& actions, Action positive, Action negative) noexcept {
    return (actions.is_held(positive) ? 1.0 : 0.0) - (actions.is_held(negative) ? 1.0 : 0.0);
}

void throttle_commands(const sim::VesselView& vessel, const ActionState& actions, double dt_s,
                       FlightControlState& state, std::vector<SignalCommand>& commands) {
    const auto throttle = find_signal(vessel, k_throttle);
    if (!throttle.has_value()) {
        state.throttle.reset();
        return;
    }
    const sim::SignalView& signal = throttle->get();
    const auto set = [&](double value) {
        state.throttle = std::clamp(value, signal.minimum, signal.maximum);
        commands.push_back(
            {.vessel = vessel.id, .signal = std::string{k_throttle}, .value = *state.throttle});
    };
    const double change = axis(actions, Action::ThrottleUp, Action::ThrottleDown);
    if (actions.was_pressed(Action::ThrottleFull)) {
        set(signal.maximum);
    } else if (actions.was_pressed(Action::ThrottleCut)) {
        set(signal.minimum);
    } else if (change != 0.0) {
        set(state.throttle.value_or(signal.value)
            + change * k_throttle_rate_per_s * dt_s * (signal.maximum - signal.minimum));
    } else {
        state.throttle.reset();
    }
}

// Commands for three signals that take a vector in the vessel's axes, asked for in the seat's.
// The pilot's hold on them lasts as long as something is asked, and is released after.
void vector_commands(const sim::VesselView& vessel, const math::Vector3& in_seat,
                     const math::Matrix3& seat_to_vessel, const std::array<std::string_view, 3>& signals,
                     bool& held, std::vector<SignalCommand>& commands) {
    const bool asked = in_seat.x != 0.0 || in_seat.y != 0.0 || in_seat.z != 0.0;
    if (!asked && !held) {
        return;
    }
    const math::Vector3 in_vessel = seat_to_vessel * in_seat;
    const std::array<double, 3> components{in_vessel.x, in_vessel.y, in_vessel.z};
    for (std::size_t index = 0; index < signals.size(); ++index) {
        const auto signal = find_signal(vessel, signals.at(index));
        if (!signal.has_value()) {
            continue;
        }
        commands.push_back(
            {.vessel = vessel.id,
             .signal = std::string{signals.at(index)},
             .value = std::clamp(components.at(index), signal->get().minimum, signal->get().maximum),
             .release = !asked});
    }
    held = asked;
}

void steering_commands(const sim::VesselView& vessel, const ActionState& actions,
                       const math::Matrix3& seat_to_vessel, FlightControlState& state,
                       std::vector<SignalCommand>& commands) {
    // The turn asked for, as a rotation vector in the seat's axes (forward, left, up): rolling
    // to the right is about +forward, the nose going down about +left, yawing left about +up.
    vector_commands(vessel,
                    {axis(actions, Action::RollRight, Action::RollLeft),
                     axis(actions, Action::PitchDown, Action::PitchUp),
                     axis(actions, Action::YawLeft, Action::YawRight)},
                    seat_to_vessel, k_attitude, state.steering, commands);
    // The push asked for, along the seat's axes.
    vector_commands(vessel,
                    {axis(actions, Action::TranslateForward, Action::TranslateBack),
                     axis(actions, Action::TranslateLeft, Action::TranslateRight),
                     axis(actions, Action::TranslateUp, Action::TranslateDown)},
                    seat_to_vessel, k_translation, state.translating, commands);
}

} // namespace

std::vector<SignalCommand> flight_commands(const sim::VesselView& vessel, const ActionState& actions,
                                           const math::Matrix3& seat_to_vessel, double dt_s,
                                           FlightControlState& state) {
    std::vector<SignalCommand> commands;
    throttle_commands(vessel, actions, dt_s, state, commands);
    steering_commands(vessel, actions, seat_to_vessel, state, commands);
    if (actions.was_pressed(Action::Stage)) {
        if (const auto stage = find_signal(vessel, vessel::k_signal_stage);
            stage.has_value() && stage->get().value < stage->get().maximum) {
            commands.push_back({.vessel = vessel.id,
                                .signal = std::string{vessel::k_signal_stage},
                                .value = stage->get().value + 1.0});
        }
    }
    if (actions.was_pressed(Action::ToggleAttitudeHold)) {
        if (const auto hold = find_signal(vessel, vessel::k_signal_attitude_hold)) {
            commands.push_back({.vessel = vessel.id,
                                .signal = std::string{vessel::k_signal_attitude_hold},
                                .value = hold->get().value >= 0.5 ? 0.0 : 1.0});
        }
    }
    return commands;
}

std::vector<SignalCommand> press_instrument(const sim::VesselView& vessel,
                                            const vessel::Instrument& instrument) {
    using Kind = vessel::Instrument::Kind;
    std::vector<SignalCommand> commands;
    const auto signal = find_signal(vessel, instrument.signal);
    if (instrument.kind == Kind::Switch && signal.has_value()) {
        const sim::SignalView& view = signal->get();
        const bool on = view.value >= 0.5 * (view.minimum + view.maximum);
        commands.push_back(
            {.vessel = vessel.id, .signal = view.name, .value = on ? view.minimum : view.maximum});
    }
    if (instrument.kind != Kind::Button) {
        return commands;
    }
    // Rationale: all or nothing. A button whose vessel lacks one of its signals would
    // otherwise half-apply (a pointing frame without its direction).
    for (const vessel::InstrumentCommand& command : instrument.commands) {
        if (!find_signal(vessel, command.signal).has_value()) {
            return {};
        }
    }
    for (const vessel::InstrumentCommand& command : instrument.commands) {
        commands.push_back({.vessel = vessel.id, .signal = command.signal, .value = command.value});
    }
    if (signal.has_value() && instrument.step != 0.0) {
        const sim::SignalView& view = signal->get();
        commands.push_back({.vessel = vessel.id,
                            .signal = view.name,
                            .value = std::clamp(view.value + instrument.step, view.minimum, view.maximum)});
    }
    return commands;
}

std::optional<double> lever_fraction(const sim::VesselView& vessel, const vessel::Instrument& lever) {
    const auto signal = find_signal(vessel, lever.signal);
    if (!signal.has_value() || !(signal->get().maximum > signal->get().minimum)) {
        return std::nullopt;
    }
    const sim::SignalView& view = signal->get();
    return std::clamp((view.value - view.minimum) / (view.maximum - view.minimum), 0.0, 1.0);
}

std::optional<SignalCommand> move_lever(const sim::VesselView& vessel, const vessel::Instrument& lever,
                                        double fraction) {
    const auto signal = find_signal(vessel, lever.signal);
    if (lever.kind != vessel::Instrument::Kind::Lever || !signal.has_value()) {
        return std::nullopt;
    }
    const sim::SignalView& view = signal->get();
    return SignalCommand{.vessel = vessel.id,
                         .signal = view.name,
                         .value =
                             view.minimum + std::clamp(fraction, 0.0, 1.0) * (view.maximum - view.minimum)};
}

} // namespace helios::app
