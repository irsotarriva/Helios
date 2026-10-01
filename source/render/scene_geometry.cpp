#include "helios/render/scene_geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>

namespace helios::render {

namespace {

using math::Vector3;

struct NamedColour {
    std::string_view name;
    std::uint32_t abgr;
};

constexpr std::array k_stock_colours{
    NamedColour{"Sun", rgba(255, 214, 120)},     NamedColour{"Mercury", rgba(158, 150, 142)},
    NamedColour{"Venus", rgba(232, 200, 138)},   NamedColour{"Earth", rgba(70, 130, 220)},
    NamedColour{"Moon", rgba(190, 190, 190)},    NamedColour{"Mars", rgba(200, 90, 50)},
    NamedColour{"Jupiter", rgba(216, 180, 140)}, NamedColour{"Saturn", rgba(227, 207, 154)},
    NamedColour{"Uranus", rgba(158, 216, 224)},  NamedColour{"Neptune", rgba(90, 120, 225)},
    NamedColour{"Pluto", rgba(201, 184, 161)},
};

constexpr std::array k_trajectory_colours{rgba(255, 170, 40), rgba(80, 220, 120), rgba(200, 110, 255),
                                          rgba(255, 90, 110)};

[[nodiscard]] std::uint32_t with_alpha(std::uint32_t abgr, std::uint8_t alpha) noexcept {
    return (abgr & 0x00FFFFFFU) | (static_cast<std::uint32_t>(alpha) << 24U);
}

void append_polyline(std::vector<LineVertex>& lines, const std::vector<Vector3>& points_m, bool closed,
                     const Vector3& camera_m, std::uint32_t abgr) {
    if (points_m.size() < 2) {
        return;
    }
    const std::size_t segments = closed ? points_m.size() : points_m.size() - 1;
    for (std::size_t index = 0; index < segments; ++index) {
        const Vector3& start_m = points_m[index];
        const Vector3& end_m = points_m[(index + 1) % points_m.size()];
        lines.push_back({to_camera_space(start_m, camera_m), abgr});
        lines.push_back({to_camera_space(end_m, camera_m), abgr});
    }
}

[[nodiscard]] Matrix4f model_matrix(const math::Matrix3& body_to_universe, double radius_m,
                                    const Float3& centre) {
    Matrix4f model{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            model.at((column * 4) + row) = static_cast<float>(body_to_universe(row, column) * radius_m);
        }
    }
    model[12] = centre.x;
    model[13] = centre.y;
    model[14] = centre.z;
    model[15] = 1.0F;
    return model;
}

} // namespace

std::uint32_t body_colour(std::string_view name) noexcept {
    for (const NamedColour& entry : k_stock_colours) {
        if (entry.name == name) {
            return entry.abgr;
        }
    }
    // FNV-1a of the name → a muted but distinct colour.
    std::uint32_t hash = 2'166'136'261U;
    for (const char character : name) {
        hash = (hash ^ static_cast<std::uint8_t>(character)) * 16'777'619U;
    }
    const auto channel = [&](unsigned shift) {
        return static_cast<std::uint8_t>(110U + ((hash >> shift) & 0x7FU));
    };
    return rgba(channel(0U), channel(8U), channel(16U));
}

FrameGeometry build_frame_geometry(const sim::SceneSnapshot& snapshot, const Vector3& camera_from_focus_m,
                                   const GeometryOptions& options) {
    FrameGeometry geometry;
    const Vector3& camera_m = camera_from_focus_m;

    // The star lights everything: the root of the domain hierarchy (the Sun, for the stock
    // universe).
    std::optional<Vector3> star_m;
    std::optional<std::uint32_t> star_index;
    for (const sim::BodyView& body : snapshot.bodies) {
        if (!body.domain_parent.has_value()) {
            star_m = body.position_m;
            star_index = body.id.index;
            break;
        }
    }

    for (const sim::BodyView& body : snapshot.bodies) {
        const Float3 centre = to_camera_space(body.position_m, camera_m);
        Vector3 to_star = star_m.has_value() ? *star_m - body.position_m : Vector3{1.0, 0.0, 0.0};
        const double to_star_norm_m = math::norm(to_star);
        to_star = to_star_norm_m > 0.0 ? to_star / to_star_norm_m : Vector3{1.0, 0.0, 0.0};
        const std::uint32_t colour = body_colour(body.name);
        geometry.bodies.push_back(
            BodyInstance{.body_index = body.id.index,
                         .centre = centre,
                         .radius_m = static_cast<float>(body.radius_m),
                         .model = model_matrix(body.body_to_universe, body.radius_m, centre),
                         .sun_direction = {static_cast<float>(to_star.x), static_cast<float>(to_star.y),
                                           static_cast<float>(to_star.z)},
                         .abgr = colour,
                         .emissive = star_index == body.id.index,
                         .distance_m = math::norm(body.position_m - camera_m)});
        geometry.labels.push_back(
            Label{.text = body.name, .position = centre, .abgr = colour, .kind = LabelKind::Body});
    }

    std::size_t trajectory_segment = 0;
    std::optional<std::uint32_t> previous_owner;
    for (const sim::LineView& line : snapshot.lines) {
        if (line.kind == sim::LineKind::BodyOrbit) {
            const auto centre = std::ranges::find(snapshot.bodies, line.frame_body, &sim::BodyView::id);
            if (centre != snapshot.bodies.end()) {
                double extent_m = 0.0;
                for (const Vector3& point_m : line.points_m) {
                    extent_m = std::max(extent_m, math::norm(point_m - centre->position_m));
                }
                const double distance_m = std::max(math::norm(centre->position_m - camera_m), 1.0);
                if (extent_m / distance_m < options.min_orbit_angular_size) {
                    continue;
                }
            }
            const auto owner =
                std::ranges::find(snapshot.bodies, bodies::BodyId{line.owner}, &sim::BodyView::id);
            const std::uint32_t colour =
                owner != snapshot.bodies.end() ? body_colour(owner->name) : rgba(200, 200, 200);
            append_polyline(geometry.lines, line.points_m, line.closed, camera_m, with_alpha(colour, 150));
        } else {
            trajectory_segment = previous_owner == line.owner ? trajectory_segment + 1 : 0;
            previous_owner = line.owner;
            const std::uint32_t colour =
                k_trajectory_colours.at(trajectory_segment % k_trajectory_colours.size());
            append_polyline(geometry.lines, line.points_m, false, camera_m, colour);
        }
    }

    for (const sim::VesselView& vessel : snapshot.vessels) {
        geometry.labels.push_back(Label{
            .text = vessel.name,
            .position = to_camera_space(vessel.position_m, camera_m),
            .abgr = vessel.status == sim::VesselStatus::Flying ? rgba(255, 255, 255) : rgba(255, 80, 80),
            .kind = LabelKind::Vessel});
    }
    return geometry;
}

} // namespace helios::render
