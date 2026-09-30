#ifndef HELIOS_RENDER_SCENE_GEOMETRY_HPP
#define HELIOS_RENDER_SCENE_GEOMETRY_HPP

#include "helios/math/vector3.hpp"
#include "helios/render/camera.hpp"
#include "helios/sim/scene_snapshot.hpp"

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace helios::render {

// Colours are packed ABGR (bgfx's Uint8×4 COLOR0 on little-endian): 0xAABBGGRR.
[[nodiscard]] constexpr std::uint32_t rgba(std::uint8_t red, std::uint8_t green, std::uint8_t blue,
                                           std::uint8_t alpha = 255) noexcept {
    return (static_cast<std::uint32_t>(alpha) << 24U) | (static_cast<std::uint32_t>(blue) << 16U)
           | (static_cast<std::uint32_t>(green) << 8U) | static_cast<std::uint32_t>(red);
}

// A stock colour for well-known bodies, otherwise a stable colour derived from the name.
[[nodiscard]] std::uint32_t body_colour(std::string_view name) noexcept;

struct LineVertex {
    Float3 position; // camera space
    std::uint32_t abgr = 0;
};

struct BodyInstance {
    std::uint32_t body_index = 0;
    Float3 centre; // camera space
    float radius_m = 0.0F;
    Matrix4f model{};     // unit sphere → camera space (rotation · scale, then translation)
    Float3 sun_direction; // unit vector from the body towards the star, universe axes
    std::uint32_t abgr = 0;
    bool emissive = false;   // the star itself
    double distance_m = 0.0; // from the camera to the centre
};

enum class LabelKind : std::uint8_t { Body, Vessel, Focus };

struct Label {
    std::string text;
    Float3 position; // camera space
    std::uint32_t abgr = 0;
    LabelKind kind = LabelKind::Body;
};

// Everything one frame of the map view draws, in camera-space floats.
struct FrameGeometry {
    std::vector<LineVertex> lines; // a line list: vertices pairwise
    std::vector<BodyInstance> bodies;
    std::vector<Label> labels;
};

struct GeometryOptions {
    // Orbit lines whose apparent size (orbit extent / distance to its centre) is below this are
    // skipped: moons' orbits vanish when the view pulls back to the whole system.
    double min_orbit_angular_size = 0.004;
};

// Converts a snapshot to camera-space floats (floating origin: see to_camera_space).
[[nodiscard]] FrameGeometry build_frame_geometry(const sim::SceneSnapshot& snapshot,
                                                 const math::Vector3& camera_from_focus_m,
                                                 const GeometryOptions& options = {});

} // namespace helios::render

#endif // HELIOS_RENDER_SCENE_GEOMETRY_HPP
