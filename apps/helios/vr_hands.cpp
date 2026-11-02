#include "vr_hands.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace helios::app {

bool Squeeze::update(double value) noexcept {
    const bool was_closed = closed_;
    if (value >= k_close) {
        closed_ = true;
    } else if (value < k_open) {
        closed_ = false;
    }
    return closed_ && !was_closed;
}

bool Press::update(bool down) noexcept {
    const bool went_down = down && !down_;
    down_ = down;
    return went_down;
}

double SnapTurn::update(double stick_x) noexcept {
    if (std::abs(stick_x) < k_centred) {
        centred_ = true;
        return 0.0;
    }
    if (!centred_ || std::abs(stick_x) < k_push) {
        return 0.0;
    }
    centred_ = false;
    // Pushed to the right, the pilot turns to the right: a negative turn to the left.
    return stick_x > 0.0 ? -k_step_rad : k_step_rad;
}

math::Vector3 fingertip_m(const Controller& controller) noexcept {
    return controller.position_m
           + math::rotate(controller.orientation, math::Vector3{k_fingertip_m, 0.0, 0.0});
}

math::Vector3 in_head_axes(const math::Vector3& vector, const math::Quaternion& head) noexcept {
    return math::rotate(math::conjugate(head), vector);
}

math::Vector3 head_to_camera_space(const math::Vector3& vector,
                                   const math::Matrix3& universe_to_camera) noexcept {
    // Forward is the camera's backward reversed, left its right reversed.
    const math::Vector3 in_camera{-vector.y, vector.z, -vector.x};
    return math::transpose(universe_to_camera) * in_camera;
}

math::Matrix3 hand_to_camera_space(const math::Quaternion& hand_in_head,
                                   const math::Matrix3& universe_to_camera) noexcept {
    math::Matrix3 columns;
    const std::array<math::Vector3, 3> axes{math::Vector3{1.0, 0.0, 0.0}, math::Vector3{0.0, 1.0, 0.0},
                                            math::Vector3{0.0, 0.0, 1.0}};
    for (std::size_t column = 0; column < axes.size(); ++column) {
        const math::Vector3 axis =
            head_to_camera_space(math::rotate(hand_in_head, axes.at(column)), universe_to_camera);
        columns(0, column) = axis.x;
        columns(1, column) = axis.y;
        columns(2, column) = axis.z;
    }
    return columns;
}

math::Vector3 arm_from_hand(const math::Vector3& moved_m, double dt_s) noexcept {
    return dt_s > 0.0 ? -(moved_m / dt_s) : math::Vector3{};
}

math::Vector3 stick_to_move(const Controller& controller) noexcept {
    return {std::clamp(controller.stick_y, -1.0, 1.0), -std::clamp(controller.stick_x, -1.0, 1.0), 0.0};
}

} // namespace helios::app
