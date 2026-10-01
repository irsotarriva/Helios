#include "helios/vessel/control_bus.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <utility>

namespace helios::vessel {

namespace {

using core::ErrorCode;

// A chain of followers longer than this is a mistake in the vessel's wiring.
constexpr int k_max_link_depth = 8;

} // namespace

core::Result<SignalId> ControlBus::add_command(std::string name, std::string unit,
                                               const CommandRange& range) {
    if (name.empty() || find(name).has_value()) {
        return core::fail(ErrorCode::InvalidArgument,
                          std::format("signal name '{}' is empty or duplicated", name));
    }
    if (!std::isfinite(range.minimum) || !std::isfinite(range.maximum) || !(range.minimum < range.maximum)
        || !(range.default_value >= range.minimum) || !(range.default_value <= range.maximum)) {
        return core::fail(ErrorCode::OutOfRange,
                          std::format("signal '{}' needs minimum <= default <= maximum", name));
    }
    signals_.push_back(Signal{.name = std::move(name),
                              .unit = std::move(unit),
                              .kind = Signal::Kind::Command,
                              .minimum = range.minimum,
                              .maximum = range.maximum,
                              .default_value = range.default_value,
                              .toggle = range.toggle,
                              .leader = {},
                              .commands = {},
                              .reported = 0.0});
    return SignalId{static_cast<std::uint32_t>(signals_.size() - 1)};
}

core::Result<SignalId> ControlBus::add_telemetry(std::string name, std::string unit) {
    if (name.empty() || find(name).has_value()) {
        return core::fail(ErrorCode::InvalidArgument,
                          std::format("signal name '{}' is empty or duplicated", name));
    }
    signals_.push_back(Signal{.name = std::move(name),
                              .unit = std::move(unit),
                              .kind = Signal::Kind::Telemetry,
                              .minimum = 0.0,
                              .maximum = 0.0,
                              .default_value = 0.0,
                              .toggle = false,
                              .leader = {},
                              .commands = {},
                              .reported = 0.0});
    return SignalId{static_cast<std::uint32_t>(signals_.size() - 1)};
}

core::VoidResult ControlBus::link(SignalId leader, SignalId follower) {
    if (leader.index >= signals_.size() || follower.index >= signals_.size()
        || signals_[leader.index].kind != Signal::Kind::Command
        || signals_[follower.index].kind != Signal::Kind::Command) {
        return core::fail(ErrorCode::InvalidArgument, "only two existing commands can be linked");
    }
    // Following the leaders from `leader` must not come back to the follower.
    std::optional<SignalId> ancestor = leader;
    for (int depth = 0; ancestor.has_value(); ++depth) {
        if (*ancestor == follower || depth >= k_max_link_depth) {
            return core::fail(ErrorCode::InvalidArgument,
                              std::format("linking '{}' to '{}' would make a loop or too long a chain",
                                          signals_[follower.index].name, signals_[leader.index].name));
        }
        ancestor = signals_[ancestor->index].leader;
    }
    signals_[follower.index].leader = leader;
    return {};
}

core::Result<SignalId> ControlBus::find(std::string_view name) const {
    const auto match = std::ranges::find(signals_, name, &Signal::name);
    if (match == signals_.end()) {
        return core::fail(ErrorCode::InvalidArgument, std::format("the vessel has no signal '{}'", name));
    }
    return SignalId{static_cast<std::uint32_t>(match - signals_.begin())};
}

core::VoidResult ControlBus::publish(SignalId id, ControlSource source, double value) {
    if (id.index >= signals_.size() || signals_[id.index].kind != Signal::Kind::Command) {
        return core::fail(ErrorCode::InvalidArgument, "only a command signal can be commanded");
    }
    if (!std::isfinite(value)) {
        return core::fail(ErrorCode::NotFinite,
                          std::format("the command for '{}' is not finite", signals_[id.index].name));
    }
    Signal& signal = signals_[id.index];
    signal.commands.at(std::to_underlying(source)) = std::clamp(value, signal.minimum, signal.maximum);
    return {};
}

core::VoidResult ControlBus::release(SignalId id, ControlSource source) {
    if (id.index >= signals_.size() || signals_[id.index].kind != Signal::Kind::Command) {
        return core::fail(ErrorCode::InvalidArgument, "only a command signal can be released");
    }
    signals_[id.index].commands.at(std::to_underlying(source)).reset();
    return {};
}

core::VoidResult ControlBus::report(SignalId id, double value) {
    if (id.index >= signals_.size() || signals_[id.index].kind != Signal::Kind::Telemetry) {
        return core::fail(ErrorCode::InvalidArgument, "only a telemetry signal can be reported");
    }
    signals_[id.index].reported = value;
    return {};
}

double ControlBus::value(SignalId id) const noexcept {
    if (id.index >= signals_.size()) {
        return 0.0;
    }
    if (signals_[id.index].kind == Signal::Kind::Telemetry) {
        return signals_[id.index].reported;
    }
    // Up the chain of leaders (kept finite by link()) to the first signal that has a command of
    // its own or no leader, then back down through the range of every follower.
    std::array<std::uint32_t, k_max_link_depth + 1> chain{};
    std::size_t length = 0;
    double result = 0.0;
    for (std::optional<SignalId> current = id; current.has_value() && length < chain.size();) {
        const Signal& signal = signals_[current->index];
        chain.at(length++) = current->index;
        const auto command =
            std::ranges::find_if(signal.commands.rbegin(), signal.commands.rend(),
                                 [](const std::optional<double>& held) { return held.has_value(); });
        if (command != signal.commands.rend()) {
            result = **command;
            break;
        }
        result = signal.default_value;
        current = signal.leader;
    }
    while (length-- > 0) {
        const Signal& signal = signals_[chain.at(length)];
        result = std::clamp(result, signal.minimum, signal.maximum);
    }
    return result;
}

void ControlBus::adopt_commands(const ControlBus& other) {
    for (const Signal& theirs : other.signals_) {
        const auto mine = std::ranges::find(signals_, theirs.name, &Signal::name);
        if (mine != signals_.end() && mine->kind == Signal::Kind::Command
            && theirs.kind == Signal::Kind::Command) {
            mine->commands = theirs.commands;
        }
    }
}

} // namespace helios::vessel
