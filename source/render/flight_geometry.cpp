#include "helios/render/flight_geometry.hpp"

#include "helios/math/quaternion.hpp"
#include "helios/vessel/cockpit.hpp"
#include "helios/vessel/part_datasheet.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <functional>
#include <numbers>
#include <string_view>
#include <utility>

namespace helios::render {

namespace {

using math::Matrix3;
using math::Vector3;

// Parts of other vessels are drawn out to here; beyond it they are far below a pixel.
constexpr double k_part_draw_distance_m = 200e3;
constexpr float k_cabin_light = 0.75F;  // the interior is lit by the cabin, mostly
constexpr float k_outside_light = 0.0F; // the outside by the star alone
constexpr std::uint8_t k_part_grey = 185;

// The ground patch.
constexpr int k_terrain_rings = 40;
constexpr int k_terrain_segments = 72;
constexpr double k_least_height_m = 1.0;
constexpr double k_least_cell_m = 10.0;
constexpr double k_greatest_cell_rad = std::numbers::pi / 18.0; // 10°

// Instruments, as fractions of their size unless in metres.
constexpr double k_dial_depth_m = 0.008;
constexpr double k_dial_sweep_rad = 1.5 * std::numbers::pi;
constexpr std::array<double, 5> k_dial_ticks{0.0, 0.25, 0.5, 0.75, 1.0};
constexpr double k_plate_depth_m = 0.006;
constexpr double k_handle_reach_m = 0.03; // how far a lever's handle stands out of its slot
constexpr double k_label_gap_m = 0.035;

constexpr std::uint32_t k_panel_dark = rgba(28, 30, 36);
constexpr std::uint32_t k_marking = rgba(190, 190, 195);
constexpr std::uint32_t k_needle = rgba(255, 150, 40);
constexpr std::uint32_t k_dead = rgba(90, 90, 95);
constexpr std::uint32_t k_lever_handle = rgba(205, 60, 50);
constexpr std::uint32_t k_switch_on = rgba(80, 220, 110);
constexpr std::uint32_t k_switch_off = rgba(150, 150, 155);
constexpr std::uint32_t k_button = rgba(170, 120, 40);
constexpr std::uint32_t k_button_active = rgba(255, 215, 110);

// Local axes → universe axes, with the origin given relative to the camera.
struct Placement {
    Matrix3 rotation;
    Vector3 origin_m;

    [[nodiscard]] Vector3 point(const Vector3& local_m) const noexcept {
        return rotation * local_m + origin_m;
    }
    [[nodiscard]] Vector3 direction(const Vector3& local) const noexcept { return rotation * local; }
};

[[nodiscard]] Float3 to_float(const Vector3& vector) noexcept {
    return {static_cast<float>(vector.x), static_cast<float>(vector.y), static_cast<float>(vector.z)};
}

[[nodiscard]] Vector3 unit_or(const Vector3& vector, const Vector3& fallback) noexcept {
    const double length = math::norm(vector);
    return length > 0.0 ? vector / length : fallback;
}

[[nodiscard]] Matrix3 with_rows(const Vector3& first, const Vector3& second, const Vector3& third) noexcept {
    return Matrix3{{first.x, first.y, first.z, second.x, second.y, second.z, third.x, third.y, third.z}};
}

// The unit shape's x, y and z edges become `x`, `y` and `z` (camera-space axes, with their
// lengths), and its centre `centre`.
[[nodiscard]] Matrix4f model_of(const Vector3& x, const Vector3& y, const Vector3& z,
                                const Vector3& centre) noexcept {
    Matrix4f model{};
    const std::array<Vector3, 4> columns{x, y, z, centre};
    for (std::size_t column = 0; column < columns.size(); ++column) {
        model.at(column * 4) = static_cast<float>(columns.at(column).x);
        model.at((column * 4) + 1) = static_cast<float>(columns.at(column).y);
        model.at((column * 4) + 2) = static_cast<float>(columns.at(column).z);
    }
    model[15] = 1.0F;
    return model;
}

[[nodiscard]] Placement vessel_placement(const sim::VesselView& vessel, const Vector3& camera_m) noexcept {
    return {.rotation = math::to_matrix(vessel.orientation), .origin_m = vessel.position_m - camera_m};
}

[[nodiscard]] Placement part_placement(const Placement& vessel, const sim::PartView& part) noexcept {
    return {.rotation = vessel.rotation * part.orientation, .origin_m = vessel.point(part.position_m)};
}

[[nodiscard]] Placement seat_placement(const Placement& part, const vessel::Cockpit& cockpit) noexcept {
    return {.rotation = part.rotation * cockpit.seat_to_part(), .origin_m = part.point(cockpit.eye_m)};
}

[[nodiscard]] std::optional<std::reference_wrapper<const sim::SignalView>>
find_signal(const sim::VesselView& vessel, std::string_view name) {
    const auto match = std::ranges::find(vessel.signals, name, &sim::SignalView::name);
    if (match == vessel.signals.end()) {
        return std::nullopt;
    }
    return std::cref(*match);
}

// A part's colour: grey, tinted by its kind so that neighbours can be told apart.
[[nodiscard]] std::uint32_t part_colour(std::string_view datasheet_id) noexcept {
    const std::uint32_t tint = body_colour(datasheet_id);
    const auto blend = [&](unsigned shift) {
        return static_cast<std::uint8_t>((((tint >> shift) & 0xFFU) + (3U * k_part_grey)) / 4U);
    };
    return rgba(blend(0U), blend(8U), blend(16U));
}

[[nodiscard]] std::uint32_t box_colour(const Vector3& colour) noexcept {
    const auto channel = [](double value) {
        return static_cast<std::uint8_t>(std::lround(std::clamp(value, 0.0, 1.0) * 255.0));
    };
    return rgba(channel(colour.x), channel(colour.y), channel(colour.z));
}

// The colour of something that shows a signal; grey when the vessel has no such signal.
[[nodiscard]] constexpr std::uint32_t if_live(bool live, std::uint32_t abgr) noexcept {
    return live ? abgr : k_dead;
}

// A number with as many decimals as its size leaves room for.
[[nodiscard]] std::string format_reading(double value, std::string_view unit) {
    const double magnitude = std::abs(value);
    int decimals = 0;
    if (magnitude < 10.0) {
        decimals = 2;
    } else if (magnitude < 100.0) {
        decimals = 1;
    }
    // Rationale: a value that rounds to zero would otherwise read "-0.00".
    const double shown = magnitude < 0.5 * std::pow(10.0, -decimals) ? 0.0 : value;
    return unit.empty() ? std::format("{:.{}f}", shown, decimals)
                        : std::format("{:.{}f} {}", shown, decimals, unit);
}

// Builds the solids of the instruments of one cockpit.
class PanelBuilder {
public:
    PanelBuilder(FlightGeometry& geometry, const Placement& seat, const sim::VesselView& vessel) noexcept
        : geometry_(geometry), seat_(seat), vessel_(vessel) {}

    void add(const vessel::Instrument& instrument, std::size_t index) {
        using Kind = vessel::Instrument::Kind;
        const auto signal = find_signal(vessel_, instrument.signal);
        // Seen by whoever faces the panel: `right` is to their right.
        const Vector3 out = instrument.facing;
        const Vector3 up = instrument.up;
        const Vector3 right = math::cross(up, out);
        const double size_m = instrument.size_m;
        InstrumentHandle handle{.instrument = index,
                                .centre = to_float(seat_.point(instrument.position_m)),
                                .radius_m = static_cast<float>(0.5 * size_m),
                                .travel = {},
                                .label_position = {},
                                .reading_position = {},
                                .label = instrument.label,
                                .reading = {},
                                .live = signal.has_value(),
                                .handled = instrument.kind != Kind::Dial};
        double label_offset_m = (0.5 * size_m) + k_label_gap_m;
        switch (instrument.kind) {
        case Kind::Dial: {
            const double radius_m = 0.5 * size_m;
            solid(SolidShape::Cylinder, instrument.position_m, k_dial_depth_m * out, radius_m * right,
                  radius_m * up, k_panel_dark);
            const auto direction_at = [&](double fraction) {
                const double angle_rad = (fraction - 0.5) * k_dial_sweep_rad;
                return std::cos(angle_rad) * up + std::sin(angle_rad) * right;
            };
            for (const double tick : k_dial_ticks) {
                const Vector3 direction = direction_at(tick);
                solid(SolidShape::Box,
                      instrument.position_m + (0.88 * radius_m) * direction + k_dial_depth_m * out,
                      (0.16 * radius_m) * direction, (0.05 * radius_m) * math::cross(out, direction),
                      0.004 * out, k_marking);
            }
            if (signal.has_value()) {
                const double value = signal->get().value;
                const double fraction = std::clamp(
                    (value - instrument.minimum) / (instrument.maximum - instrument.minimum), 0.0, 1.0);
                const Vector3 direction = direction_at(fraction);
                solid(SolidShape::Box,
                      instrument.position_m + (0.38 * radius_m) * direction + (k_dial_depth_m + 0.004) * out,
                      (0.8 * radius_m) * direction, (0.08 * radius_m) * math::cross(out, direction),
                      0.006 * out, k_needle);
                handle.reading = format_reading(value * instrument.display_scale, instrument.display_unit);
            }
            break;
        }
        case Kind::Lever: {
            solid(SolidShape::Box, instrument.position_m, 0.025 * right, size_m * up, k_plate_depth_m * out,
                  k_panel_dark);
            double fraction = 0.0;
            if (signal.has_value()) {
                const sim::SignalView& view = signal->get();
                fraction =
                    view.maximum > view.minimum
                        ? std::clamp((view.value - view.minimum) / (view.maximum - view.minimum), 0.0, 1.0)
                        : 0.0;
                handle.reading = std::format("{:.0f} %", 100.0 * fraction);
            }
            const Vector3 grip_m =
                instrument.position_m + ((fraction - 0.5) * size_m) * up + k_handle_reach_m * out;
            solid(SolidShape::Box, grip_m, 0.07 * right, 0.03 * up, (2.0 * k_handle_reach_m) * out,
                  if_live(signal.has_value(), k_lever_handle));
            handle.centre = to_float(seat_.point(grip_m));
            handle.radius_m = 0.06F;
            handle.travel = to_float(seat_.direction(size_m * up));
            break;
        }
        case Kind::Switch: {
            solid(SolidShape::Box, instrument.position_m, size_m * right, (1.6 * size_m) * up,
                  k_plate_depth_m * out, k_panel_dark);
            const bool on = signal.has_value()
                            && signal->get().value >= 0.5 * (signal->get().minimum + signal->get().maximum);
            solid(SolidShape::Box, instrument.position_m + ((on ? 0.4 : -0.4) * size_m) * up + 0.015 * out,
                  (0.5 * size_m) * right, (0.6 * size_m) * up, 0.03 * out,
                  if_live(signal.has_value(), on ? k_switch_on : k_switch_off));
            handle.radius_m = static_cast<float>(size_m);
            label_offset_m = (0.8 * size_m) + k_label_gap_m;
            break;
        }
        case Kind::Button: {
            // Lit while everything it commands is as it would set it.
            bool active = !instrument.commands.empty();
            bool live = instrument.commands.empty() ? signal.has_value() : true;
            for (const vessel::InstrumentCommand& command : instrument.commands) {
                const auto commanded = find_signal(vessel_, command.signal);
                live = live && commanded.has_value();
                active = active && commanded.has_value() && commanded->get().value == command.value;
            }
            handle.live = live;
            solid(SolidShape::Box, instrument.position_m + 0.01 * out, size_m * right, size_m * up,
                  0.02 * out, if_live(live, active ? k_button_active : k_button));
            handle.radius_m = static_cast<float>(0.7 * size_m);
            break;
        }
        }
        handle.label_position =
            to_float(seat_.point(instrument.position_m + label_offset_m * up + 0.01 * out));
        handle.reading_position =
            to_float(seat_.point(instrument.position_m - label_offset_m * up + 0.01 * out));
        geometry_.get().instruments.push_back(std::move(handle));
    }

private:
    // `x`, `y`, `z`: what the unit shape's edges become, in the seat's axes. They must be
    // right-handed (x × y along z), or the solid would be drawn inside out.
    void solid(SolidShape shape, const Vector3& centre_m, const Vector3& x, const Vector3& y,
               const Vector3& z, std::uint32_t abgr) {
        geometry_.get().solids.push_back(
            SolidInstance{.shape = shape,
                          .model = model_of(seat_.direction(x), seat_.direction(y), seat_.direction(z),
                                            seat_.point(centre_m)),
                          .abgr = abgr,
                          .cabin_light = k_cabin_light});
    }

    std::reference_wrapper<FlightGeometry> geometry_;
    Placement seat_;
    std::reference_wrapper<const sim::VesselView> vessel_;
};

// The largest of 1, 2 and 5 times a power of ten that is not above `value`.
[[nodiscard]] double round_down_125(double value) noexcept {
    const double decade = std::pow(10.0, std::floor(std::log10(value)));
    const double mantissa = value / decade;
    if (mantissa >= 5.0) {
        return 5.0 * decade;
    }
    return (mantissa >= 2.0 ? 2.0 : 1.0) * decade;
}

[[nodiscard]] std::optional<TerrainPatch> terrain_patch(const sim::BodyView& body, const Vector3& camera_m) {
    const Vector3 from_centre_m = camera_m - body.position_m;
    const double distance_m = math::norm(from_centre_m);
    const double radius_m = body.radius_m;
    if (!(radius_m > 0.0) || !(distance_m > 0.0)) {
        return std::nullopt;
    }
    // Rationale: a camera at or under the surface (a wreck) still gets ground just below it.
    const double height_m = std::max(distance_m - radius_m, k_least_height_m);
    if (height_m >= radius_m) {
        return std::nullopt; // far enough for the sphere mesh
    }
    const double eye_radius_m = radius_m + height_m;
    const Vector3 up = from_centre_m / distance_m;
    const Vector3 first = unit_or(math::cross(Vector3{0.0, 0.0, 1.0}, up), Vector3{1.0, 0.0, 0.0});
    const Vector3 second = math::cross(up, first);

    // The grid: cells of the body's own latitude and longitude, about as wide as the camera
    // is high. Along a parallel a cell spans more longitude the nearer the pole, in steps of
    // two so that the lines stay where they are while the camera moves.
    const Matrix3 universe_to_body = math::transpose(body.body_to_universe);
    const auto latitude_longitude = [&](const Vector3& direction) {
        const Vector3 fixed = universe_to_body * direction;
        return std::pair{std::asin(std::clamp(fixed.z, -1.0, 1.0)), std::atan2(fixed.y, fixed.x)};
    };
    const double cell_rad =
        std::min(round_down_125(std::max(2.0 * height_m, k_least_cell_m)) / radius_m, k_greatest_cell_rad);
    const auto [latitude_rad, longitude_rad] = latitude_longitude(up);
    const double widening = std::exp2(std::ceil(std::log2(1.0 / std::max(std::cos(latitude_rad), 1e-3))));
    const double longitude_cell_rad = cell_rad * widening;
    const double latitude_origin_rad = cell_rad * std::round(latitude_rad / cell_rad);
    const double longitude_origin_rad = longitude_cell_rad * std::round(longitude_rad / longitude_cell_rad);

    TerrainPatch patch{.mesh = {}, .abgr = body_colour(body.name), .cell_m = cell_rad * radius_m};
    const auto vertex_at = [&](double angle_rad, double azimuth_rad) {
        const Vector3 across = std::cos(azimuth_rad) * first + std::sin(azimuth_rad) * second;
        const Vector3 direction = std::cos(angle_rad) * up + std::sin(angle_rad) * across;
        // Rationale: R cos θ − (R + h) written without the cancellation of two large numbers.
        const double half_sine = std::sin(0.5 * angle_rad);
        const Vector3 position_m = (-height_m - 2.0 * radius_m * half_sine * half_sine) * up
                                   + (radius_m * std::sin(angle_rad)) * across;
        const auto [latitude, longitude] = latitude_longitude(direction);
        const double longitude_offset_rad =
            std::remainder(longitude - longitude_origin_rad, 2.0 * std::numbers::pi);
        return MeshVertex{.position = to_float(position_m),
                          .normal = to_float(direction),
                          .u = static_cast<float>(longitude_offset_rad / longitude_cell_rad),
                          .v = static_cast<float>((latitude - latitude_origin_rad) / cell_rad)};
    };
    // Rings from a fraction of the height below the camera out to just past the horizon, each
    // wider than the last by the same factor.
    const double horizon_rad = std::acos(radius_m / eye_radius_m);
    const double outermost_rad = std::min(1.02 * horizon_rad, std::numbers::pi);
    const double innermost_rad = std::min(0.25 * height_m / radius_m, outermost_rad / k_terrain_rings);
    const double growth = std::pow(outermost_rad / innermost_rad, 1.0 / (k_terrain_rings - 1));
    patch.mesh.vertices.reserve(1 + static_cast<std::size_t>(k_terrain_rings * k_terrain_segments));
    patch.mesh.vertices.push_back(vertex_at(0.0, 0.0));
    double angle_rad = innermost_rad;
    for (int ring = 0; ring < k_terrain_rings; ++ring) {
        for (int segment = 0; segment < k_terrain_segments; ++segment) {
            patch.mesh.vertices.push_back(
                vertex_at(angle_rad, 2.0 * std::numbers::pi * segment / k_terrain_segments));
        }
        angle_rad *= growth;
    }
    const auto index_of = [](int ring, int segment) {
        return static_cast<std::uint16_t>(1 + (ring * k_terrain_segments) + (segment % k_terrain_segments));
    };
    for (int segment = 0; segment < k_terrain_segments; ++segment) {
        patch.mesh.indices.insert(patch.mesh.indices.end(),
                                  {std::uint16_t{0}, index_of(0, segment), index_of(0, segment + 1)});
    }
    for (int ring = 0; ring + 1 < k_terrain_rings; ++ring) {
        for (int segment = 0; segment < k_terrain_segments; ++segment) {
            const std::uint16_t inner = index_of(ring, segment);
            const std::uint16_t inner_next = index_of(ring, segment + 1);
            const std::uint16_t outer = index_of(ring + 1, segment);
            const std::uint16_t outer_next = index_of(ring + 1, segment + 1);
            patch.mesh.indices.insert(patch.mesh.indices.end(),
                                      {inner, outer, outer_next, inner, outer_next, inner_next});
        }
    }
    return patch;
}

} // namespace

void HeadPose::turn(double yaw_delta_rad, double pitch_delta_rad) noexcept {
    yaw_rad = std::clamp(yaw_rad + yaw_delta_rad, -k_max_yaw_rad, k_max_yaw_rad);
    pitch_rad = std::clamp(pitch_rad + pitch_delta_rad, -k_max_pitch_rad, k_max_pitch_rad);
}

std::optional<std::size_t> seat_part(const sim::VesselView& vessel) noexcept {
    for (std::size_t index = 0; index < vessel.parts.size(); ++index) {
        if (vessel.parts[index].datasheet != nullptr && vessel.parts[index].datasheet->cockpit.has_value()) {
            return index;
        }
    }
    return std::nullopt;
}

std::optional<Matrix3> seat_to_vessel(const sim::VesselView& vessel) {
    const auto part = seat_part(vessel);
    if (!part.has_value()) {
        return std::nullopt;
    }
    const sim::PartView& view = vessel.parts[*part];
    const std::optional<vessel::Cockpit>& cockpit = view.datasheet->cockpit;
    if (!cockpit.has_value()) {
        return std::nullopt;
    }
    return view.orientation * cockpit->seat_to_part();
}

std::optional<ViewPoint> seat_view_point(const sim::VesselView& vessel, const HeadPose& head) {
    const auto part = seat_part(vessel);
    if (!part.has_value()) {
        return std::nullopt;
    }
    const sim::PartView& view = vessel.parts[*part];
    const std::optional<vessel::Cockpit>& cockpit = view.datasheet->cockpit;
    if (!cockpit.has_value()) {
        return std::nullopt;
    }
    const Placement seat =
        seat_placement(part_placement(vessel_placement(vessel, Vector3{}), view), *cockpit);
    const Vector3 forward = seat.direction({1.0, 0.0, 0.0});
    const Vector3 left = seat.direction({0.0, 1.0, 0.0});
    const Vector3 up = seat.direction({0.0, 0.0, 1.0});
    const Vector3 gaze =
        std::cos(head.pitch_rad) * (std::cos(head.yaw_rad) * forward + std::sin(head.yaw_rad) * left)
        + std::sin(head.pitch_rad) * up;
    const Vector3 right = unit_or(math::cross(gaze, up), -1.0 * left);
    return ViewPoint{.position_from_focus_m = seat.origin_m,
                     .universe_to_camera = with_rows(right, math::cross(right, gaze), -1.0 * gaze)};
}

ViewPoint chase_view_point(const sim::SceneSnapshot& snapshot, const sim::VesselView& vessel,
                           const OrbitCamera& camera) {
    Vector3 up{0.0, 0.0, 1.0};
    if (vessel.domain.index < snapshot.bodies.size()) {
        up = unit_or(vessel.position_m - snapshot.bodies[vessel.domain.index].position_m, up);
    }
    const Vector3 east = unit_or(math::cross(Vector3{0.0, 0.0, 1.0}, up), Vector3{1.0, 0.0, 0.0});
    const Vector3 north = math::cross(up, east);
    const Vector3 backward =
        std::cos(camera.pitch_rad) * (std::cos(camera.yaw_rad) * east + std::sin(camera.yaw_rad) * north)
        + std::sin(camera.pitch_rad) * up;
    const Vector3 right = unit_or(math::cross(up, backward), east);
    return ViewPoint{.position_from_focus_m = vessel.position_m + camera.distance_m * backward,
                     .universe_to_camera = with_rows(right, math::cross(backward, right), backward)};
}

FlightGeometry build_flight_geometry(const sim::SceneSnapshot& snapshot, std::size_t vessel,
                                     const ViewPoint& view, bool from_seat) {
    const Vector3& camera_m = view.position_from_focus_m;
    FlightGeometry geometry;
    geometry.scene = build_frame_geometry(snapshot, camera_m);
    geometry.scene.lines.clear(); // orbits belong to the map
    geometry.sun_direction = {1.0F, 0.0F, 0.0F};
    if (vessel >= snapshot.vessels.size()) {
        return geometry;
    }
    const sim::VesselView& flown = snapshot.vessels[vessel];
    for (const sim::BodyView& body : snapshot.bodies) {
        if (!body.domain_parent.has_value()) {
            geometry.sun_direction =
                to_float(unit_or(body.position_m - flown.position_m, Vector3{1.0, 0.0, 0.0}));
            break;
        }
    }

    // The ground of the body the vessel is near replaces that body's sphere: the sphere's
    // facets are kilometres off the surface, the patch is exact below the camera.
    if (flown.domain.index < snapshot.bodies.size()) {
        const sim::BodyView& ground = snapshot.bodies[flown.domain.index];
        if (ground.domain_parent.has_value()) { // not for the star
            geometry.terrain = terrain_patch(ground, camera_m);
        }
        if (geometry.terrain.has_value()) {
            std::erase_if(geometry.scene.bodies,
                          [&](const BodyInstance& body) { return body.body_index == ground.id.index; });
        }
    }

    const std::optional<std::size_t> seat = from_seat ? seat_part(flown) : std::nullopt;
    for (std::size_t index = 0; index < snapshot.vessels.size(); ++index) {
        const sim::VesselView& drawn = snapshot.vessels[index];
        if (math::norm(drawn.position_m - camera_m) > k_part_draw_distance_m) {
            continue;
        }
        const Placement placement = vessel_placement(drawn, camera_m);
        for (std::size_t part = 0; part < drawn.parts.size(); ++part) {
            const sim::PartView& part_view = drawn.parts[part];
            if (part_view.datasheet == nullptr) {
                continue;
            }
            const Placement part_frame = part_placement(placement, part_view);
            const std::optional<vessel::Cockpit>& interior = part_view.datasheet->cockpit;
            if (index == vessel && seat == part && interior.has_value()) {
                // From inside: the cabin, not the hull around it.
                const vessel::Cockpit& cockpit = *interior;
                const Placement seat_frame = seat_placement(part_frame, cockpit);
                for (const vessel::CockpitBox& box : cockpit.boxes) {
                    geometry.solids.push_back(
                        SolidInstance{.shape = SolidShape::Box,
                                      .model = model_of(seat_frame.direction({box.size_m.x, 0.0, 0.0}),
                                                        seat_frame.direction({0.0, box.size_m.y, 0.0}),
                                                        seat_frame.direction({0.0, 0.0, box.size_m.z}),
                                                        seat_frame.point(box.centre_m)),
                                      .abgr = box_colour(box.colour),
                                      .cabin_light = k_cabin_light});
                }
                PanelBuilder panel(geometry, seat_frame, drawn);
                for (std::size_t instrument = 0; instrument < cockpit.instruments.size(); ++instrument) {
                    panel.add(cockpit.instruments[instrument], instrument);
                }
                continue;
            }
            if (const auto& shape = part_view.datasheet->shape; shape.has_value()) {
                geometry.solids.push_back(SolidInstance{
                    .shape = SolidShape::Cylinder,
                    .model = model_of(part_frame.direction({shape->length_m, 0.0, 0.0}),
                                      part_frame.direction({0.0, shape->radius_m, 0.0}),
                                      part_frame.direction({0.0, 0.0, shape->radius_m}), part_frame.origin_m),
                    .abgr = part_colour(part_view.datasheet->id),
                    .cabin_light = k_outside_light});
            }
        }
    }
    return geometry;
}

Float3 ray_through_pixel(float x_px, float y_px, float width_px, float height_px,
                         double vertical_field_of_view_rad, const Matrix3& universe_to_camera) noexcept {
    const double half_height = std::tan(0.5 * vertical_field_of_view_rad);
    const double aspect = static_cast<double>(width_px) / std::max(static_cast<double>(height_px), 1.0);
    const Vector3 in_camera{((2.0 * x_px / std::max(width_px, 1.0F)) - 1.0) * half_height * aspect,
                            (1.0 - (2.0 * y_px / std::max(height_px, 1.0F))) * half_height, -1.0};
    return to_float(unit_or(math::transpose(universe_to_camera) * in_camera, Vector3{1.0, 0.0, 0.0}));
}

std::optional<std::size_t> pick_instrument(std::span<const InstrumentHandle> instruments,
                                           const Float3& ray_direction) noexcept {
    std::optional<std::size_t> nearest;
    float nearest_m = 0.0F;
    for (std::size_t index = 0; index < instruments.size(); ++index) {
        const InstrumentHandle& handle = instruments[index];
        if (!handle.handled || !handle.live) {
            continue;
        }
        const Float3& centre = handle.centre;
        const float along_m =
            (centre.x * ray_direction.x) + (centre.y * ray_direction.y) + (centre.z * ray_direction.z);
        if (along_m <= 0.0F) {
            continue; // behind the camera
        }
        const float off_x = centre.x - (along_m * ray_direction.x);
        const float off_y = centre.y - (along_m * ray_direction.y);
        const float off_z = centre.z - (along_m * ray_direction.z);
        const float miss_m = std::sqrt((off_x * off_x) + (off_y * off_y) + (off_z * off_z));
        if (miss_m <= handle.radius_m && (!nearest.has_value() || along_m < nearest_m)) {
            nearest = index;
            nearest_m = along_m;
        }
    }
    return nearest;
}

double lever_travel_fraction(const InstrumentHandle& lever, const Matrix4f& view_projection, float width_px,
                             float height_px, float dx_px, float dy_px) noexcept {
    const Float3 low{lever.centre.x - (0.5F * lever.travel.x), lever.centre.y - (0.5F * lever.travel.y),
                     lever.centre.z - (0.5F * lever.travel.z)};
    const Float3 high{low.x + lever.travel.x, low.y + lever.travel.y, low.z + lever.travel.z};
    const auto from = project_to_screen(low, view_projection, width_px, height_px);
    const auto to = project_to_screen(high, view_projection, width_px, height_px);
    if (!from.has_value() || !to.has_value()) {
        return 0.0;
    }
    const double along_x = to->x - from->x;
    const double along_y = to->y - from->y;
    const double length_squared = (along_x * along_x) + (along_y * along_y);
    // Rationale: a lever seen end-on has no direction on the screen to drag it along.
    if (length_squared < 1.0) {
        return 0.0;
    }
    return ((dx_px * along_x) + (dy_px * along_y)) / length_squared;
}

} // namespace helios::render
