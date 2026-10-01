#ifndef HELIOS_APPS_HELIOS_FLIGHT_CONTROLS_HPP
#define HELIOS_APPS_HELIOS_FLIGHT_CONTROLS_HPP

#include "helios/math/matrix3.hpp"
#include "helios/sim/scene_snapshot.hpp"
#include "helios/vessel/cockpit.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// The pilot's inputs as commands on the control bus (BRIEFING §8.3, §12.1 rule 4). Keys, the
// mouse on a cockpit lever and, later, a hand in VR all end up here, as actions or as an
// instrument being handled; nothing below this layer knows which device it was.
namespace helios::app {

// A value for one signal of a vessel's control bus, or the end of the pilot's hold on it.
struct SignalCommand {
    sim::VesselId vessel;
    std::string signal;
    double value = 0.0;
    bool release = false; // withdraw the pilot's command, so that whatever is below it applies again
};

enum class Action : std::uint8_t {
    ThrottleUp,
    ThrottleDown,
    ThrottleFull,
    ThrottleCut,
    PitchUp, // the nose towards the pilot's head
    PitchDown,
    YawLeft,
    YawRight,
    RollLeft,
    RollRight,
    Stage,
    ToggleAttitudeHold,
};
inline constexpr std::size_t k_action_count = 12;

// Which actions are being asked for this frame.
struct ActionState {
    std::array<bool, k_action_count> held{};    // for as long as the key is down
    std::array<bool, k_action_count> pressed{}; // once, when it goes down

    [[nodiscard]] bool is_held(Action action) const noexcept {
        return held.at(static_cast<std::size_t>(action));
    }
    [[nodiscard]] bool was_pressed(Action action) const noexcept {
        return pressed.at(static_cast<std::size_t>(action));
    }
    void set(Action action, bool is_held_now, bool was_pressed_now) noexcept {
        held.at(static_cast<std::size_t>(action)) = is_held_now;
        pressed.at(static_cast<std::size_t>(action)) = was_pressed_now;
    }
};

// What has to be remembered between frames.
struct FlightControlState {
    // The throttle while its keys are held: the snapshot lags the keys by a tick or two.
    std::optional<double> throttle;
    bool steering = false; // the pilot holds the attitude signals
};

inline constexpr double k_throttle_rate_per_s = 0.5;

// The commands the held and pressed actions amount to for `vessel`, over a frame of `dt_s`.
// Steering is asked for in the seat's axes and commanded in the vessel's (`seat_to_vessel`;
// the identity for a vessel flown from outside). The pilot's hold on the attitude signals
// lasts as long as a steering key is down, and is released after.
[[nodiscard]] std::vector<SignalCommand> flight_commands(const sim::VesselView& vessel,
                                                         const ActionState& actions,
                                                         const math::Matrix3& seat_to_vessel, double dt_s,
                                                         FlightControlState& state);

// A switch thrown or a button pressed. Nothing for a dial, a lever or a dead instrument.
[[nodiscard]] std::vector<SignalCommand> press_instrument(const sim::VesselView& vessel,
                                                          const vessel::Instrument& instrument);

// Where a lever stands, from 0 to 1; none if the vessel lacks its signal.
[[nodiscard]] std::optional<double> lever_fraction(const sim::VesselView& vessel,
                                                   const vessel::Instrument& lever);

// A lever set to `fraction` of its travel.
[[nodiscard]] std::optional<SignalCommand> move_lever(const sim::VesselView& vessel,
                                                      const vessel::Instrument& lever, double fraction);

} // namespace helios::app

#endif // HELIOS_APPS_HELIOS_FLIGHT_CONTROLS_HPP
