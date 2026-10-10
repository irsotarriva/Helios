#ifndef HELIOS_VESSEL_COCKPIT_HPP
#define HELIOS_VESSEL_COCKPIT_HPP

#include "helios/core/error.hpp"
#include "helios/core/toml.hpp"
#include "helios/math/matrix3.hpp"
#include "helios/math/vector3.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// The interior of a crewed part (BRIEFING §12, D26): a seat, the solids around it, and the
// instruments the pilot reads and handles. Everything but the seat is given in the seat's own
// axes, with the origin at the seated pilot's eyes: +x is where the pilot faces, +y is to the
// pilot's left, +z is up.
namespace helios::vessel {

// One solid piece of the interior: a box with its edges along the seat's axes.
struct CockpitBox {
    math::Vector3 centre_m;
    math::Vector3 size_m;
    math::Vector3 colour{0.3, 0.3, 0.32}; // red, green, blue in [0, 1]
    bool glass = false;                   // a window: it stops a body and is not drawn
};

// Something loose in a cabin, to pick up, carry and throw: a box that starts at rest where it
// is put.
struct CockpitItem {
    std::string name;
    math::Vector3 position_m; // of its centre
    math::Vector3 size_m;
    double mass_kg = 1.0;
    math::Vector3 colour{0.55, 0.42, 0.25};
};

// Where the pilot of a cabin goes out and comes back in (BRIEFING §12, D31): the place where
// the inside of the vessel is exchanged for the outside, behind the time it takes to put the
// suit on and pump the air out.
struct Airlock {
    math::Vector3 inside_m;  // where the pilot stands to use it
    math::Vector3 outside_m; // where the pilot is on leaving it, clear of the hull
    double cycle_s = 20.0;   // to suit up and let the air out, or the reverse
};

struct InstrumentCommand {
    std::string signal;
    double value = 0.0;
};

// Something on a panel, bound to the vessel's control bus by signal name (BRIEFING §8.3). The
// names are those of the vessel, so an instrument written against "engine/throttle" works in
// any vessel that has an engine; in one that has no such signal it is dead.
struct Instrument {
    enum class Kind : std::uint8_t {
        Lever,  // sets a command signal anywhere in its range
        Switch, // sets a command signal to its minimum or its maximum
        Button, // publishes its commands, or steps a command signal, when pressed
        Dial,   // shows any signal
    };

    Kind kind = Kind::Dial;
    std::string label;
    std::string signal; // none for a button that only publishes `commands`
    math::Vector3 position_m;
    math::Vector3 facing{-1.0, 0.0, 0.0}; // out of the panel, towards whoever uses it (unit)
    math::Vector3 up{0.0, 0.0, 1.0};      // a lever's direction of increase, a dial's top (unit)
    double size_m = 0.1;                  // a lever's travel, a dial's diameter, a switch's or button's width
    // Dial: the range of its needle, and how the number under it is written.
    double minimum = 0.0;
    double maximum = 1.0;
    double display_scale = 1.0; // the number shown is the signal's value times this
    std::string display_unit;
    // Button.
    std::vector<InstrumentCommand> commands;
    double step = 0.0; // added to `signal` on each press (the staging button: 1)
};

struct Cockpit {
    math::Vector3 eye_m;                  // the seated pilot's eyes, part axes
    math::Vector3 forward{1.0, 0.0, 0.0}; // where the pilot faces, part axes (unit)
    math::Vector3 up{0.0, 0.0, 1.0};      // part axes (unit, at right angles to `forward`)
    // Whether the pilot can leave the seat: the boxes then enclose a cabin to move about in.
    bool walkable = false;
    std::vector<CockpitBox> boxes;
    std::vector<Instrument> instruments;
    std::vector<CockpitItem> items; // only in a cabin (`walkable`)
    std::optional<Airlock> airlock; // only in a cabin

    // Takes the seat's axes to the part's: its columns are forward, left and up.
    [[nodiscard]] math::Matrix3 seat_to_part() const noexcept;
};

// Reads the `cockpit` table of a part (the format is described in data/parts/README.md) and
// checks it: directions are normalised, and `up` is made exactly perpendicular to `forward`.
[[nodiscard]] core::Result<Cockpit> parse_cockpit(const core::TomlValue& table);

} // namespace helios::vessel

#endif // HELIOS_VESSEL_COCKPIT_HPP
