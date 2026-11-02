#ifndef HELIOS_APPS_HELIOS_PILOT_CONTROLS_HPP
#define HELIOS_APPS_HELIOS_PILOT_CONTROLS_HPP

#include "helios/math/matrix3.hpp"
#include "helios/math/quaternion.hpp"
#include "helios/math/vector3.hpp"

// Where the pilot looks while moving about a cabin (BRIEFING §12, D27). The player owns this,
// not the simulation: it follows the mouse (later, the headset) without waiting for a tick, and
// is sent to the simulation as part of the pilot's input.
namespace helios::app {

struct PilotLook {
    static constexpr double k_max_pitch_rad = 1.45;
    // How fast a body that has weight turns its head end up, radians per second.
    static constexpr double k_upright_rate_rad_s = 4.0;

    // Takes the body's axes (forward, left, up) to the vessel's (on foot outside: the ground's).
    math::Quaternion body;
    // The head, above the body's own level; only used while there is weight, when the body
    // stays upright. A floating body pitches as a whole.
    double pitch_rad = 0.0;
};

// Takes the eyes' axes (forward, left, up) to the vessel's.
[[nodiscard]] math::Quaternion view_of(const PilotLook& look) noexcept;

// The look of a pilot getting to their feet outside (BRIEFING D32), facing the way the suit
// does: `seat_to_ground` takes the helmet's axes (forward, left, up) to the ground's.
[[nodiscard]] PilotLook look_from_suit(const math::Matrix3& seat_to_ground) noexcept;

// The look of a pilot rising from a seat, with the head turned as it was in the seat.
[[nodiscard]] PilotLook look_from_seat(const math::Matrix3& seat_to_vessel, double head_yaw_rad,
                                       double head_pitch_rad) noexcept;

// Turns the look: `yaw_rad` to the left, `pitch_rad` up, `roll_rad` to the right. `up` is the
// direction against the weight felt, in the axes the look is in, or zero when the pilot floats:
// - floating, the body turns freely about all three of its own axes;
// - with weight, it turns about its own up, the head pitches, and the body rights itself
//   towards `up` over `dt_s` (rolling is not asked of someone standing).
void turn(PilotLook& look, double yaw_rad, double pitch_rad, double roll_rad, const math::Vector3& up,
          double dt_s) noexcept;

// The hand moved over the surface it holds by a drag of the mouse (D27): `dx_px` to the right
// and `dy_px` down the picture, as how far the body moves relative to the hand, in the pilot's
// axes (forward, left, up), per pixel. `normal` is the surface's, out of it, in the same axes.
// Dragging towards where the surface lies on the picture pushes against it, and sends the body
// out along the normal; away from it pulls the body in. Across that the hand slides along the
// surface, and the body goes the other way. A surface faced squarely lies "up" the picture:
// dragging up pushes it away, as on a mouse mat.
[[nodiscard]] math::Vector3 drag_to_arm(double dx_px, double dy_px, const math::Vector3& normal) noexcept;

} // namespace helios::app

#endif // HELIOS_APPS_HELIOS_PILOT_CONTROLS_HPP
