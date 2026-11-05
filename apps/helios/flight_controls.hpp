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
// control being handled; nothing below this layer knows which device it was.
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
    // Pushing the vessel along the seat's axes without turning it (a suit's thruster pack).
    TranslateForward,
    TranslateBack,
    TranslateLeft,
    TranslateRight,
    TranslateUp,
    TranslateDown,
};
inline constexpr std::size_t k_action_count = 18;

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
    bool steering = false;    // the pilot holds the attitude signals
    bool translating = false; // ... and the translation signals
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

// A control of the cockpit in the pilot's hand (BRIEFING D43), from when it is taken to when it
// is let go. The mouse and a headset's hands both end up here.
struct ControlHold {
    std::size_t control = 0;            // its index in the cockpit
    double fraction = 0.0;              // an axis: where it is held, from 0 to 1
    double dragged = 0.0;               // a switch or an encoder: a drag not yet a whole step
    std::array<double, 3> deflection{}; // a stick: each way, from -1 to 1
};

// The rest position past which a sprung axis or a detent takes a lever let go nearby.
inline constexpr double k_detent_reach = 0.03;

// Taking hold of a control, at `side` of it (-1 to 1 along its travel; see render::ControlPick):
// a push-button is pressed, an unlocked switch goes to its next position that way (a two-way one
// flips), an encoder clicks a step that way. A locked switch only moves when dragged (pulled out
// and over). `hold` is set up for what follows.
[[nodiscard]] std::vector<SignalCommand>
take_control(const sim::VesselView& vessel, const vessel::Control& control, double side, ControlHold& hold);

// The hand holding it moves: `along` and `along_up` are how far, as fractions of the handle's
// travel and travel_up (render::ControlHandle).
[[nodiscard]] std::vector<SignalCommand> drag_control(const sim::VesselView& vessel,
                                                      const vessel::Control& control, double along,
                                                      double along_up, ControlHold& hold);

// Letting go: a push-button's release, a sprung switch back to rest, a lever into a detent or to
// where a spring takes it, a stick centred and the pilot's hold on its signals released.
[[nodiscard]] std::vector<SignalCommand>
let_go_control(const sim::VesselView& vessel, const vessel::Control& control, const ControlHold& hold);

} // namespace helios::app

#endif // HELIOS_APPS_HELIOS_FLIGHT_CONTROLS_HPP
