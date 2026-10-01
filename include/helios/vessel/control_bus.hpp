#ifndef HELIOS_VESSEL_CONTROL_BUS_HPP
#define HELIOS_VESSEL_CONTROL_BUS_HPP

#include "helios/core/error.hpp"

#include <array>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace helios::vessel {

// Who is commanding a signal, in increasing priority (BRIEFING §8.3): the pilot overrides the
// autopilot, which overrides the vessel's own sequencer (staging).
enum class ControlSource : std::uint8_t {
    Sequencer,
    Autopilot,
    Pilot,
};
inline constexpr std::size_t k_control_source_count = 3;

struct SignalId {
    std::uint32_t index = 0;

    [[nodiscard]] friend constexpr auto operator<=>(const SignalId&, const SignalId&) noexcept = default;
};

struct Signal {
    enum class Kind : std::uint8_t {
        Command,   // something the vessel is told: a throttle, a switch
        Telemetry, // something the vessel reports: a thrust, a temperature
    };

    std::string name;
    std::string unit;
    Kind kind = Kind::Command;
    double minimum = 0.0; // commands are clamped to [minimum, maximum]
    double maximum = 1.0;
    double default_value = 0.0;
    bool toggle = false;            // a switch rather than a lever (for whatever draws the control)
    std::optional<SignalId> leader; // a command with no source of its own follows this signal
    std::array<std::optional<double>, k_control_source_count> commands; // per ControlSource
    double reported = 0.0;                                              // Telemetry
};

// The control bus of one vessel (BRIEFING §8.3, D6): every input and every reading is a named
// signal. A key binding, a cockpit switch, an autopilot and a remote player all publish to the
// same signals, and gauges read the same telemetry, so none of them needs to know which parts
// the vessel is made of. Arbitration between sources lives here and nowhere else.
class ControlBus {
public:
    struct CommandRange {
        double minimum = 0.0;
        double maximum = 1.0;
        double default_value = 0.0;
        bool toggle = false;
    };

    [[nodiscard]] core::Result<SignalId> add_command(std::string name, std::string unit,
                                                     const CommandRange& range);
    [[nodiscard]] core::Result<SignalId> add_telemetry(std::string name, std::string unit);

    // Makes `follower` take `leader`'s value (clamped to its own range) unless a source of the
    // same or a higher priority commands it directly: "every engine's throttle follows the
    // vessel's".
    [[nodiscard]] core::VoidResult link(SignalId leader, SignalId follower);

    [[nodiscard]] core::Result<SignalId> find(std::string_view name) const;
    [[nodiscard]] std::span<const Signal> signals() const noexcept { return signals_; }

    // A source's command stays until that source changes or releases it.
    [[nodiscard]] core::VoidResult publish(SignalId id, ControlSource source, double value);
    [[nodiscard]] core::VoidResult release(SignalId id, ControlSource source);
    [[nodiscard]] core::VoidResult report(SignalId id, double value);

    // A command's value: that of the highest-priority source commanding it or a signal it
    // follows (its own command standing against a leader's from the same source), else the
    // default. A telemetry signal's last report. 0 for an unknown id.
    [[nodiscard]] double value(SignalId id) const noexcept;

    // Takes over the commands of the signals of `other` that exist here under the same name:
    // what a vessel keeps of its controls when it is rebuilt after losing or gaining parts.
    void adopt_commands(const ControlBus& other);

private:
    std::vector<Signal> signals_;
};

} // namespace helios::vessel

#endif // HELIOS_VESSEL_CONTROL_BUS_HPP
