#ifndef HELIOS_VESSEL_CONTROLS_HPP
#define HELIOS_VESSEL_CONTROLS_HPP

#include "helios/core/error.hpp"
#include "helios/core/toml.hpp"
#include "helios/math/vector3.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// What a cockpit's controls and indicators do, kept apart from how they look (BRIEFING D43), as
// a web page's markup is from its style sheet. A control is one of five types, by what the
// hand does to it and what that makes of the vessel's control bus; how it looks is a `Look`,
// taken from style sheets (`data/styles/*.toml`) by type and class, and from the control's own
// `look`. The formats are in data/parts/README.md and data/styles/README.md.
namespace helios::vessel {

// A value for one signal of the vessel's control bus (BRIEFING §8.3).
struct SignalSetting {
    std::string signal;
    double value = 0.0;
};

// How a control or an indicator looks: a built-in shape and its measures. What `travel` means
// depends on the shape (data/styles/README.md): a length for a lever, an angle for a toggle or
// a dial.
struct Look {
    std::string shape;
    double size_m = 0.05;
    double travel = 0.0;
    math::Vector3 colour{0.2, 0.21, 0.24}; // its body
    math::Vector3 accent{0.9, 0.9, 0.86};  // what moves or lights: a handle, a needle, a lamp lit
    math::Vector3 paint{0.86, 0.86, 0.82}; // its label
};

// Part of a look, as a style sheet's rule or a control's own `look` gives it.
struct LookPatch {
    std::optional<std::string> shape;
    std::optional<double> size_m;
    std::optional<double> travel;
    std::optional<math::Vector3> colour;
    std::optional<math::Vector3> accent;
    std::optional<math::Vector3> paint;

    // `other`'s settings over these.
    void apply(const LookPatch& other);
};

// Where something sits in the cockpit, in the seat's axes (cockpit.hpp).
struct PanelPlace {
    math::Vector3 position_m;
    math::Vector3 facing{-1.0, 0.0, 0.0}; // out of the panel, towards whoever uses it (unit)
    math::Vector3 up{0.0, 0.0, 1.0};      // the top as lettered; a lever's or a switch's way up (unit)
};

// A cover over a control, to lift before the control can be worked. Closing it may push the
// control back (`closes_to`, a switch's position), as many real covers do.
struct Guard {
    std::optional<std::size_t> closes_to;
    Look look;
};

// One position of a switch: what it sets, and whether a spring takes it back to the switch's
// rest position when the hand lets go.
struct SwitchPosition {
    std::vector<SignalSetting> settings;
    bool spring = false;
};

struct Control {
    enum class Type : std::uint8_t {
        Momentary, // acts while pressed: a push-button
        Switch,    // stays in one of its positions: toggle, rocker, rotary selector, breaker
        Axis,      // one value in a range: a lever, a knob, a wheel
        Stick,     // two or three values, centred by a spring: a control stick, a hand controller
        Encoder,   // steps a value up or down, without an end: a heading knob
    };

    std::string id; // unique in the cockpit
    Type type = Type::Momentary;
    std::string label;
    std::vector<std::string> classes; // for the style sheets
    PanelPlace place;
    // The signal it moves: a momentary's stepped one, a switch's (positions given as numbers),
    // an axis's, an encoder's.
    std::string signal;
    std::vector<std::string> signals; // a stick's: two or three, along its right, up, and twist
    // Momentary: set when pressed, and when let go; and `step` added to `signal` on each press.
    std::vector<SignalSetting> press;
    std::vector<SignalSetting> release;
    double step = 0.0; // also an encoder's change per click
    // Switch.
    std::vector<SwitchPosition> positions; // from the lowest (or the most anticlockwise)
    std::size_t rest = 0;                  // where a spring takes it back to
    bool locked = false;                   // pulled out before it can be moved
    // Axis: fractions of the travel where it snaps when let go nearby, and where a spring
    // takes it when let go.
    std::vector<double> detents;
    std::optional<double> spring_to;
    bool wrap = false; // Encoder: past its maximum it comes round to its minimum
    std::optional<Guard> guard;
    LookPatch own_look;
    Look look; // resolved from the style sheets and `own_look` when the part is catalogued
};

struct Indicator {
    enum class Type : std::uint8_t {
        Gauge,   // a value on a scale: a dial, a tape
        State,   // one of a few states: a lamp, a talkback flag
        Readout, // a value written out
    };

    std::string id; // unique in the cockpit, among controls too
    Type type = Type::Gauge;
    std::string label;
    std::vector<std::string> classes;
    PanelPlace place;
    std::string signal;
    // Gauge: the range of the scale.
    double minimum = 0.0;
    double maximum = 1.0;
    // Gauge, readout: the number written is the value times `scale`, followed by `unit`.
    double scale = 1.0;
    std::string unit;
    // State: the state is how many of these the value has reached (a lamp: one, lit from it on).
    std::vector<double> thresholds;
    LookPatch own_look;
    Look look;
};

// A style sheet (data/styles/README.md): looks by element (a control's or an indicator's type,
// or `guard`) and class.
struct StyleSheet {
    struct Rule {
        std::string element;
        std::string class_name; // empty: every element of the type
        LookPatch look;
    };

    std::string name;
    std::string extends; // empty: none
    std::vector<Rule> rules;
};

[[nodiscard]] std::string_view element_name(Control::Type type) noexcept;
[[nodiscard]] std::string_view element_name(Indicator::Type type) noexcept;
inline constexpr std::string_view k_guard_element = "guard";

// Reads a style sheet: `name` is its file's stem.
[[nodiscard]] core::Result<StyleSheet> parse_style_sheet(std::string name, const core::TomlValue& document);

// The look of an element of a cockpit in style sheet `style` (none: the built-in looks only),
// with `classes`, over which `own` is laid. Fails for an unknown sheet, a loop of `extends`, or
// a shape the element cannot have.
[[nodiscard]] core::Result<Look> resolve_look(std::span<const StyleSheet> sheets, std::string_view style,
                                              std::string_view element, std::span<const std::string> classes,
                                              const LookPatch& own);

[[nodiscard]] core::Result<Control> parse_control(const core::TomlValue& table);
[[nodiscard]] core::Result<Indicator> parse_indicator(const core::TomlValue& table);

// A switch's position whose settings hold, given the signals' values (`value_of`, none for a
// signal the vessel lacks): the first that holds all of them, or else the nearest by `signal`.
template <typename ValueOf>
[[nodiscard]] std::optional<std::size_t> switch_position(const Control& control, const ValueOf& value_of) {
    std::optional<std::size_t> nearest;
    double nearest_gap = 0.0;
    for (std::size_t index = 0; index < control.positions.size(); ++index) {
        bool holds = !control.positions[index].settings.empty();
        for (const SignalSetting& setting : control.positions[index].settings) {
            const std::optional<double> value = value_of(setting.signal);
            if (!value.has_value()) {
                return std::nullopt;
            }
            holds = holds && *value == setting.value;
            if (setting.signal == control.signal) {
                const double gap = *value > setting.value ? *value - setting.value : setting.value - *value;
                if (!nearest.has_value() || gap < nearest_gap) {
                    nearest = index;
                    nearest_gap = gap;
                }
            }
        }
        if (holds) {
            return index;
        }
    }
    return nearest;
}

} // namespace helios::vessel

#endif // HELIOS_VESSEL_CONTROLS_HPP
