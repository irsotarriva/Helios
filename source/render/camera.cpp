#include "helios/render/camera.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>

namespace helios::render {

namespace {

constexpr std::size_t k_order = 4;

[[nodiscard]] constexpr std::size_t at(std::size_t row, std::size_t column) noexcept {
    return (column * k_order) + row;
}

} // namespace

math::Vector3 OrbitCamera::offset_from_focus_m() const noexcept {
    return distance_m
           * math::Vector3{std::cos(pitch_rad) * std::cos(yaw_rad), std::cos(pitch_rad) * std::sin(yaw_rad),
                           std::sin(pitch_rad)};
}

math::Matrix3 OrbitCamera::universe_to_camera() const noexcept {
    const math::Vector3 offset_m = offset_from_focus_m();
    const math::Vector3 backward = offset_m / math::norm(offset_m); // camera +z points away from the focus
    const math::Vector3 world_up{0.0, 0.0, 1.0};
    math::Vector3 right = math::cross(world_up, backward);
    right = right / math::norm(right);
    const math::Vector3 up = math::cross(backward, right);
    math::Matrix3 rotation;
    rotation.elements = {right.x, right.y, right.z, up.x, up.y, up.z, backward.x, backward.y, backward.z};
    return rotation;
}

void OrbitCamera::orbit(double yaw_delta_rad, double pitch_delta_rad) noexcept {
    yaw_rad = std::remainder(yaw_rad + yaw_delta_rad, 2.0 * std::numbers::pi);
    pitch_rad = std::clamp(pitch_rad + pitch_delta_rad, -k_max_pitch_rad, k_max_pitch_rad);
}

void OrbitCamera::zoom(double factor, double floor_m) noexcept {
    const double lower_m = std::max(k_min_distance_m, floor_m);
    distance_m = std::clamp(distance_m * factor, lower_m, k_max_distance_m);
}

Float3 to_camera_space(const math::Vector3& position_from_focus_m,
                       const math::Vector3& camera_from_focus_m) noexcept {
    const math::Vector3 relative_m = position_from_focus_m - camera_from_focus_m;
    return {static_cast<float>(relative_m.x), static_cast<float>(relative_m.y),
            static_cast<float>(relative_m.z)};
}

Matrix4f view_matrix(const math::Matrix3& universe_to_camera) noexcept {
    Matrix4f matrix{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            matrix[at(row, column)] = static_cast<float>(universe_to_camera(row, column));
        }
    }
    matrix[at(3, 3)] = 1.0F;
    return matrix;
}

Matrix4f reversed_infinite_projection(double vertical_field_of_view_rad, double aspect_ratio, double near_m,
                                      bool homogeneous_depth) noexcept {
    const double focal = 1.0 / std::tan(0.5 * vertical_field_of_view_rad);
    Matrix4f matrix{};
    matrix[at(0, 0)] = static_cast<float>(focal / aspect_ratio);
    matrix[at(1, 1)] = static_cast<float>(focal);
    // z_clip = near (or 2·near + z for [−1, 1]); w_clip = −z.
    matrix[at(2, 2)] = homogeneous_depth ? 1.0F : 0.0F;
    matrix[at(2, 3)] = static_cast<float>(homogeneous_depth ? 2.0 * near_m : near_m);
    matrix[at(3, 2)] = -1.0F;
    return matrix;
}

Matrix4f multiply(const Matrix4f& lhs, const Matrix4f& rhs) noexcept {
    Matrix4f product{};
    for (std::size_t row = 0; row < k_order; ++row) {
        for (std::size_t column = 0; column < k_order; ++column) {
            float sum = 0.0F;
            for (std::size_t inner = 0; inner < k_order; ++inner) {
                sum += lhs[at(row, inner)] * rhs[at(inner, column)];
            }
            product[at(row, column)] = sum;
        }
    }
    return product;
}

std::optional<ScreenPoint> project_to_screen(const Float3& camera_space_point,
                                             const Matrix4f& view_projection, float width_px,
                                             float height_px) noexcept {
    const std::array<float, 4> point{camera_space_point.x, camera_space_point.y, camera_space_point.z, 1.0F};
    std::array<float, 4> clip{};
    for (std::size_t row = 0; row < k_order; ++row) {
        for (std::size_t column = 0; column < k_order; ++column) {
            clip.at(row) += view_projection[at(row, column)] * point.at(column);
        }
    }
    if (clip[3] <= 0.0F) {
        return std::nullopt;
    }
    const float ndc_x = clip[0] / clip[3];
    const float ndc_y = clip[1] / clip[3];
    return ScreenPoint{.x = 0.5F * (ndc_x + 1.0F) * width_px,
                       .y = 0.5F * (1.0F - ndc_y) * height_px,
                       .depth = clip[2] / clip[3]};
}

} // namespace helios::render
