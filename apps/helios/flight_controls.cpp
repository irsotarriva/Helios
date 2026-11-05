#include "flight_controls.hpp"

#include "helios/math/vector3.hpp"
#include "helios/vessel/vessel_systems.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <span>
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

[[nodiscard]] double fraction_of(const sim::SignalView& signal) noexcept {
    return signal.maximum > signal.minimum
               ? std::clamp((signal.value - signal.minimum) / (signal.maximum - signal.minimum), 0.0, 1.0)
               : 0.0;
}

[[nodiscard]] SignalCommand at_fraction(const sim::VesselView& vessel, const sim::SignalView& signal,
                                        double fraction) {
    return {.vessel = vessel.id,
            .signal = signal.name,
            .value = signal.minimum + std::clamp(fraction, 0.0, 1.0) * (signal.maximum - signal.minimum)};
}

// Reads any signal of the vessel, command or not; none for one it lacks.
[[nodiscard]] auto value_reader(const sim::VesselView& vessel) {
    return [&vessel](const std::string& name) -> std::optional<double> {
        const auto match = std::ranges::find(vessel.signals, name, &sim::SignalView::name);
        return match != vessel.signals.end() ? std::optional<double>{match->value} : std::nullopt;
    };
}

// All or nothing: false, and nothing added, if the vessel lacks one of the signals. Rationale:
// half a setting would leave the vessel in a state nobody asked for (a pointing frame without
// its direction).
[[nodiscard]] bool settings_commands(const sim::VesselView& vessel,
                                     std::span<const vessel::SignalSetting> settings,
                                     std::vector<SignalCommand>& commands) {
    for (const vessel::SignalSetting& setting : settings) {
        if (!find_signal(vessel, setting.signal).has_value()) {
            return false;
        }
    }
    for (const vessel::SignalSetting& setting : settings) {
        commands.push_back({.vessel = vessel.id, .signal = setting.signal, .value = setting.value});
    }
    return true;
}

// A switch moved one position `way` (+1: to the next; 0: a two-way switch flipped), if it can.
void step_switch(const sim::VesselView& vessel, const vessel::Control& control, int way,
                 std::vector<SignalCommand>& commands) {
    const auto position = vessel::switch_position(control, value_reader(vessel));
    if (!position.has_value()) {
        return;
    }
    const auto last = static_cast<std::ptrdiff_t>(control.positions.size()) - 1;
    const auto at = static_cast<std::ptrdiff_t>(*position);
    const std::ptrdiff_t to = way == 0 ? last - at : std::clamp<std::ptrdiff_t>(at + way, 0, last);
    if (to != at) {
        (void)settings_commands(vessel, control.positions.at(static_cast<std::size_t>(to)).settings,
                                commands);
    }
}

// An encoder clicked one step `way`; past an end it stops there, or comes round if it wraps.
void step_encoder(const sim::VesselView& vessel, const vessel::Control& control, int way,
                  std::vector<SignalCommand>& commands) {
    const auto signal = find_signal(vessel, control.signal);
    if (!signal.has_value()) {
        return;
    }
    const sim::SignalView& view = signal->get();
    double value = view.value + way * control.step;
    const double span = view.maximum - view.minimum;
    if (control.wrap && span > 0.0 && (value > view.maximum || value < view.minimum)) {
        value = view.minimum + std::fmod(std::fmod(value - view.minimum, span) + span, span);
    }
    commands.push_back(
        {.vessel = vessel.id, .signal = view.name, .value = std::clamp(value, view.minimum, view.maximum)});
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

std::vector<SignalCommand> take_control(const sim::VesselView& vessel, const vessel::Control& control,
                                        double side, ControlHold& hold) {
    using Type = vessel::Control::Type;
    hold = ControlHold{.control = hold.control, .fraction = 0.0, .dragged = 0.0, .deflection = {}};
    std::vector<SignalCommand> commands;
    switch (control.type) {
    case Type::Momentary:
        if (!settings_commands(vessel, control.press, commands)) {
            return {};
        }
        if (control.step != 0.0) {
            if (const auto signal = find_signal(vessel, control.signal)) {
                const sim::SignalView& view = signal->get();
                commands.push_back(
                    {.vessel = vessel.id,
                     .signal = view.name,
                     .value = std::clamp(view.value + control.step, view.minimum, view.maximum)});
            }
        }
        break;
    case Type::Switch:
        if (!control.locked) {
            // A two-way switch flips wherever it is clicked.
            const int towards = side < 0.0 ? -1 : 1;
            step_switch(vessel, control, control.positions.size() == 2 ? 0 : towards, commands);
        }
        break;
    case Type::Encoder: step_encoder(vessel, control, side < 0.0 ? -1 : 1, commands); break;
    case Type::Axis:
        if (const auto signal = find_signal(vessel, control.signal)) {
            hold.fraction = fraction_of(signal->get());
        }
        break;
    case Type::Stick: break;
    }
    return commands;
}

std::vector<SignalCommand> drag_control(const sim::VesselView& vessel, const vessel::Control& control,
                                        double along, double along_up, ControlHold& hold) {
    using Type = vessel::Control::Type;
    std::vector<SignalCommand> commands;
    switch (control.type) {
    case Type::Momentary: break;
    case Type::Axis:
        hold.fraction = std::clamp(hold.fraction + along, 0.0, 1.0);
        if (const auto signal = find_signal(vessel, control.signal)) {
            commands.push_back(at_fraction(vessel, signal->get(), hold.fraction));
        }
        break;
    case Type::Switch:
    case Type::Encoder:
        // Each whole travel is one step; half of one is enough to take it.
        hold.dragged += along;
        while (std::abs(hold.dragged) >= 0.5) {
            const int way = hold.dragged > 0.0 ? 1 : -1;
            hold.dragged -= way;
            if (control.type == Type::Switch) {
                step_switch(vessel, control, way, commands);
            } else {
                step_encoder(vessel, control, way, commands);
            }
        }
        break;
    case Type::Stick: {
        const std::array<double, 3> moved{along, along_up, 0.0};
        for (std::size_t index = 0; index < control.signals.size() && index < moved.size(); ++index) {
            hold.deflection.at(index) = std::clamp(hold.deflection.at(index) + moved.at(index), -1.0, 1.0);
            if (const auto signal = find_signal(vessel, control.signals[index])) {
                commands.push_back(
                    at_fraction(vessel, signal->get(), 0.5 * (hold.deflection.at(index) + 1.0)));
            }
        }
        break;
    }
    }
    return commands;
}

std::vector<SignalCommand> let_go_control(const sim::VesselView& vessel, const vessel::Control& control,
                                          const ControlHold& hold) {
    using Type = vessel::Control::Type;
    std::vector<SignalCommand> commands;
    switch (control.type) {
    case Type::Momentary:
        if (!settings_commands(vessel, control.release, commands)) {
            return {};
        }
        break;
    case Type::Switch: {
        const auto position = vessel::switch_position(control, value_reader(vessel));
        if (position.has_value() && control.positions.at(*position).spring
            && !settings_commands(vessel, control.positions.at(control.rest).settings, commands)) {
            return {};
        }
        break;
    }
    case Type::Axis: {
        const auto signal = find_signal(vessel, control.signal);
        if (!signal.has_value()) {
            break;
        }
        std::optional<double> settles = control.spring_to;
        for (const double detent : control.detents) {
            if (!settles.has_value() && std::abs(hold.fraction - detent) <= k_detent_reach) {
                settles = detent;
            }
        }
        if (settles.has_value()) {
            commands.push_back(at_fraction(vessel, signal->get(), *settles));
        }
        break;
    }
    case Type::Stick:
        // The spring centres it; and the pilot no longer holds the signals, so the keys or
        // whatever is below the pilot apply again.
        for (const std::string& name : control.signals) {
            if (find_signal(vessel, name).has_value()) {
                commands.push_back({.vessel = vessel.id, .signal = name, .value = 0.0, .release = true});
            }
        }
        break;
    case Type::Encoder: break;
    }
    return commands;
}

} // namespace helios::app
