#ifndef HELIOS_APPS_HELIOS_KEY_BINDINGS_HPP
#define HELIOS_APPS_HELIOS_KEY_BINDINGS_HPP

#include "helios/core/error.hpp"
#include "helios/core/toml.hpp"
#include "helios/vessel/cockpit.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "flight_controls.hpp"

// Which keys do what (BRIEFING D45): a file, `data/input/bindings.toml` unless `--bindings` names
// another (its format is in data/input/README.md). A key is bound to an action, or to a control
// of whatever cockpit the pilot sits in, by the control's id. Keys are named by where they are,
// not by what they print: the key a US keyboard has there, so a binding means the same key on
// every layout. This is GPU- and window-free; the client turns its device's keys into these.
namespace helios::app {

// A key by position: its USB HID usage (keyboard page), which is also SDL's scancode.
using KeyCode = std::uint16_t;

// The key a name means ("A", "Left Shift", "Keypad 7"; case does not matter); none for a name
// that is not a key's.
[[nodiscard]] std::optional<KeyCode> key_code(std::string_view name) noexcept;
// A key's name as the bindings file writes it; empty for a key without one.
[[nodiscard]] std::string_view key_name(KeyCode key) noexcept;

// An action's name in the bindings file ("throttle_up").
[[nodiscard]] std::string_view action_name(Action action) noexcept;
[[nodiscard]] std::optional<Action> action_from_name(std::string_view name) noexcept;

struct ActionKey {
    KeyCode key = 0;
    Action action = Action::Interact;
};

// A key on a cockpit's control: while it is down the control is in the pilot's hand, as if
// taken with the mouse (flight_controls.hpp), and it is let go with the key.
struct ControlKey {
    std::string control;     // its id in the cockpit
    std::string part;        // only in a cockpit of this part (its datasheet's id); empty: any
    double rate_per_s = 0.5; // an axis: fractions of its travel a second
    // Which way: 1 up (or on, or clockwise), -1 the other way, 0 as a click in its middle (a
    // two-way switch flips). An axis moves that way at `rate` for as long as the key is held.
    int way = 0;
    KeyCode key = 0;
};

struct KeyBindings {
    std::vector<ActionKey> actions;
    std::vector<ControlKey> controls;

    // Every key bound to `action`, in the file's order.
    [[nodiscard]] std::vector<KeyCode> keys_of(Action action) const;
    // Whether `key` is one of `action`'s.
    [[nodiscard]] bool binds(KeyCode key, Action action) const noexcept;
};

// Reads a bindings file. Fails for an unknown action or key, a key bound to both an action and a
// control, or the same key twice on one action.
[[nodiscard]] core::Result<KeyBindings> parse_key_bindings(const core::TomlValue& document);
[[nodiscard]] core::Result<KeyBindings> load_key_bindings(const std::filesystem::path& file);

// The control of `cockpit` (in a part whose datasheet is `part_id`) that `binding` names; none
// when this cockpit has no such control.
[[nodiscard]] std::optional<std::size_t>
bound_control(const vessel::Cockpit& cockpit, std::string_view part_id, const ControlKey& binding) noexcept;

// What the picture's prompts and hints call the keys, as bound.
struct KeyLabels {
    std::string interact; // "F"
    std::string jump;     // "Space"
    std::string move;     // the four ways along: "arrows", or each key
    std::string forward;  // "Up"
    std::string back;     // "Down"
    std::string up_down;  // "Right Shift / Right Ctrl"
    std::string turn;     // "W S A D Q E"
    std::string flight_view;
};
[[nodiscard]] KeyLabels key_labels(const KeyBindings& bindings);

// The help panel's text, from the bindings.
[[nodiscard]] std::string help_text(const KeyBindings& bindings);

} // namespace helios::app

#endif // HELIOS_APPS_HELIOS_KEY_BINDINGS_HPP
