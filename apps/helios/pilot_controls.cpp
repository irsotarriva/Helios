#include "pilot_controls.hpp"

#include <algorithm>
#include <cmath>

namespace helios::app {

namespace {

using math::Quaternion;
using math::Vector3;

constexpr Vector3 k_forward{1.0, 0.0, 0.0};
constexpr Vector3 k_left{0.0, 1.0, 0.0};
constexpr Vector3 k_up{0.0, 0.0, 1.0};

// Looking up is a turn about the left axis that takes forward towards up: a negative angle.
[[nodiscard]] Quaternion pitched(double pitch_rad) noexcept {
    return math::from_axis_angle(k_left, -pitch_rad);
}

} // namespace

Quaternion view_of(const PilotLook& look) noexcept {
    return math::normalized(look.body * pitched(look.pitch_rad));
}

PilotLook look_from_seat(const math::Matrix3& seat_to_vessel, double head_yaw_rad,
                         double head_pitch_rad) noexcept {
    return {.body = math::normalized(math::from_matrix(seat_to_vessel)
                                     * math::from_axis_angle(k_up, head_yaw_rad)),
            .pitch_rad = std::clamp(head_pitch_rad, -PilotLook::k_max_pitch_rad, PilotLook::k_max_pitch_rad)};
}

void turn(PilotLook& look, double yaw_rad, double pitch_rad, double roll_rad, const Vector3& up,
          double dt_s) noexcept {
    const bool weightless = math::norm(up) < 0.5;
    if (weightless) {
        // The head's pitch becomes the body's, so the view does not jump when the weight goes.
        look.body =
            math::normalized(look.body * pitched(look.pitch_rad) * math::from_axis_angle(k_up, yaw_rad)
                             * pitched(pitch_rad) * math::from_axis_angle(k_forward, roll_rad));
        look.pitch_rad = 0.0;
        return;
    }
    look.body = look.body * math::from_axis_angle(k_up, yaw_rad);
    look.pitch_rad =
        std::clamp(look.pitch_rad + pitch_rad, -PilotLook::k_max_pitch_rad, PilotLook::k_max_pitch_rad);
    // Right the body: the shortest turn that takes its up to the real one, a step at a time.
    const Vector3 body_up = math::rotate(look.body, k_up);
    const Vector3 axis = math::cross(body_up, up);
    const double sine = math::norm(axis);
    const double angle_rad = std::atan2(sine, math::dot(body_up, up));
    if (angle_rad > 1e-9) {
        // Rationale: upside down exactly, every axis across the body is as short a way round.
        const Vector3 unit_axis = sine > 1e-9 ? axis / sine : math::rotate(look.body, k_forward);
        const double step_rad = std::min(angle_rad, PilotLook::k_upright_rate_rad_s * std::max(dt_s, 0.0));
        look.body = math::from_axis_angle(unit_axis, step_rad) * look.body;
    }
    look.body = math::normalized(look.body);
}

} // namespace helios::app
