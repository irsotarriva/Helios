#ifndef HELIOS_RENDER_CAMERA_HPP
#define HELIOS_RENDER_CAMERA_HPP

#include "helios/math/matrix3.hpp"
#include "helios/math/vector3.hpp"

#include <array>
#include <optional>

namespace helios::render {

// Column-major 4×4 float matrix, as bgfx and the shaders expect (element (row, column) at
// index column·4 + row).
using Matrix4f = std::array<float, 16>;

struct Float3 {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

// A camera orbiting the focus of a SceneSnapshot. Angles are in the universe axes (the J2000
// ecliptic for the stock Solar System, +z = ecliptic north): yaw about +z from +x, pitch above
// the xy-plane.
struct OrbitCamera {
    static constexpr double k_max_pitch_rad = 1.5607; // just short of the pole, to keep "up" defined
    static constexpr double k_min_distance_m = 1.0;
    static constexpr double k_max_distance_m = 1e16;

    double yaw_rad = 0.0;
    double pitch_rad = 0.3;
    double distance_m = 3e7;
    double vertical_field_of_view_rad = 0.9;

    // Camera position relative to the focus, universe axes. A double: at 1e13 m from the focus a
    // float would be off by a kilometre.
    [[nodiscard]] math::Vector3 offset_from_focus_m() const noexcept;

    // Rows are the camera's right, up and backward axes in universe coordinates (so the matrix
    // takes universe-axis vectors to camera axes; the camera looks along −z).
    [[nodiscard]] math::Matrix3 universe_to_camera() const noexcept;

    void orbit(double yaw_delta_rad, double pitch_delta_rad) noexcept;
    // Multiplies the distance by `factor`, clamped to [max(min, floor_m), max].
    void zoom(double factor, double floor_m) noexcept;
};

// Floating origin: a focus-relative double position expressed relative to the camera, and only
// then rounded to float. The rounding error is relative to the distance from the camera, never
// to the distance from the universe origin.
[[nodiscard]] Float3 to_camera_space(const math::Vector3& position_from_focus_m,
                                     const math::Vector3& camera_from_focus_m) noexcept;

// View matrix for camera-relative positions: rotation only, the camera is at the origin.
[[nodiscard]] Matrix4f view_matrix(const math::Matrix3& universe_to_camera) noexcept;

// Right-handed perspective projection with reversed depth and an infinite far plane: depth is 1
// at the near plane and tends to 0 at infinity, so depth precision (with a float depth buffer)
// stays uniform in log distance, from metres to light-years. Depth test GREATER, clear to 0.
// `homogeneous_depth` selects the [−1, 1] NDC depth range (OpenGL) instead of [0, 1].
[[nodiscard]] Matrix4f reversed_infinite_projection(double vertical_field_of_view_rad, double aspect_ratio,
                                                    double near_m, bool homogeneous_depth) noexcept;

// A field of view that need not be symmetric about the axis of the camera, as a headset gives
// one per eye: the angle from the axis to each edge, negative to the left and downwards.
struct FieldOfView {
    double left_rad = -0.7;
    double right_rad = 0.7;
    double up_rad = 0.7;
    double down_rad = -0.7;
};

// The same projection for a field of view with its axis off the centre of the picture.
[[nodiscard]] Matrix4f reversed_infinite_projection(const FieldOfView& field_of_view, double near_m,
                                                    bool homogeneous_depth) noexcept;

// View matrix for positions relative to a point the camera is displaced from: an eye of a
// headset, drawing geometry that was built once for the head. `offset_m` is the camera's place
// from that point, in universe axes.
[[nodiscard]] Matrix4f view_matrix(const math::Matrix3& universe_to_camera,
                                   const math::Vector3& offset_m) noexcept;

[[nodiscard]] Matrix4f multiply(const Matrix4f& lhs, const Matrix4f& rhs) noexcept;

struct ScreenPoint {
    float x = 0.0F; // pixels from the left
    float y = 0.0F; // pixels from the top
    float depth = 0.0F;
};

// Projects a camera-space point to window pixels; none if it is behind the camera.
[[nodiscard]] std::optional<ScreenPoint> project_to_screen(const Float3& camera_space_point,
                                                           const Matrix4f& view_projection, float width_px,
                                                           float height_px) noexcept;

} // namespace helios::render

#endif // HELIOS_RENDER_CAMERA_HPP
