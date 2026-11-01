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

PilotLook look_from_suit(const math::Matrix3& seat_to_ground) noexcept {
    return {.body = math::normalized(math::from_matrix(seat_to_ground)), .pitch_rad = 0.0};
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

Vector3 drag_to_arm(double dx_px, double dy_px, const Vector3& normal) noexcept {
    // The picture's right and up, in the pilot's axes.
    constexpr Vector3 k_right{0.0, -1.0, 0.0};
    // Rationale: a surface seen edge-on or square-on has no clear side on the picture; a bias
    // makes "up the picture" count as "into the picture", as pushing a mouse away from oneself.
    constexpr double k_into_picture = 0.25;
    const double length = math::norm(normal);
    const Vector3 out = length > 1e-9 ? normal / length : Vector3{-1.0, 0.0, 0.0};
    // Where the surface lies on the picture, seen from the hand: against its normal.
    double into_x = -math::dot(out, k_right);
    double into_y = -math::dot(out, k_up) + k_into_picture;
    const double into_length = std::hypot(into_x, into_y);
    into_x = into_length > 1e-9 ? into_x / into_length : 0.0;
    into_y = into_length > 1e-9 ? into_y / into_length : 1.0;
    // The drag, with up the picture positive.
    const double drag_x = dx_px;
    const double drag_y = -dy_px;
    const double push = (drag_x * into_x) + (drag_y * into_y);
    const Vector3 across = ((drag_x - push * into_x) * k_right) + ((drag_y - push * into_y) * k_up);
    const Vector3 slide = across - math::dot(across, out) * out;
    // Pushing the hand into the surface sends the body out; sliding it sends the body back.
    return (push * out) - slide;
}
} // namespace helios::app
