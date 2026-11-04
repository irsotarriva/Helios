#ifndef HELIOS_VESSEL_COCKPIT_HPP
#define HELIOS_VESSEL_COCKPIT_HPP

#include "helios/core/error.hpp"
#include "helios/core/toml.hpp"
#include "helios/math/matrix3.hpp"
#include "helios/math/vector3.hpp"
#include "helios/vessel/controls.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

// The interior of a crewed part (BRIEFING §12, D26): a seat, the solids around it, and the
// controls and indicators the pilot handles and reads (D43). Everything but the seat is given in the seat's
// own axes, with the origin at the seated pilot's eyes: +x is where the pilot faces, +y is to the pilot's
// left, +z is up.
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

// The way up to the hatch from the ground (BRIEFING §12, D35): a line on the hull a suited
// pilot can take hold of with a hand and climb, from the seat's axes like everything else
// about the cabin, with its origin at the seated pilot's eyes.
struct Ladder {
    math::Vector3 from_m; // the foot, where the climb ends below
    math::Vector3 to_m;   // the top, at the hatch
    math::Vector3 out_m;  // away from the hull, into which the body hangs (unit)
    double side_m = 0.0;  // how far across the hull the hand may go; 0: it stays on the line
};

struct Cockpit {
    math::Vector3 eye_m;                  // the seated pilot's eyes, part axes
    math::Vector3 forward{1.0, 0.0, 0.0}; // where the pilot faces, part axes (unit)
    math::Vector3 up{0.0, 0.0, 1.0};      // part axes (unit, at right angles to `forward`)
    // Whether the pilot can leave the seat: the boxes then enclose a cabin to move about in.
    bool walkable = false;
    std::vector<CockpitBox> boxes;
    std::string style; // the style sheet its controls and indicators are drawn in; none: built-in
    std::vector<Control> controls;
    std::vector<Indicator> indicators;
    std::vector<CockpitItem> items; // only in a cabin (`walkable`)
    std::optional<Airlock> airlock; // only in a cabin
    std::optional<Ladder> ladder;   // only in a cabin: outside it, on the hull

    // Takes the seat's axes to the part's: its columns are forward, left and up.
    [[nodiscard]] math::Matrix3 seat_to_part() const noexcept;
};

// Reads the `cockpit` table of a part (the format is described in data/parts/README.md) and
// checks it: directions are normalised, and `up` is made exactly perpendicular to `forward`.
[[nodiscard]] core::Result<Cockpit> parse_cockpit(const core::TomlValue& table);

// Gives each control, guard and indicator its look, from the cockpit's style sheet among
// `sheets` (BRIEFING D43).
[[nodiscard]] core::VoidResult resolve_looks(Cockpit& cockpit, std::span<const StyleSheet> sheets);

} // namespace helios::vessel

#endif // HELIOS_VESSEL_COCKPIT_HPP
