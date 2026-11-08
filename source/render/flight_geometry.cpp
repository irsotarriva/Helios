#include "helios/render/flight_geometry.hpp"

#include "helios/math/quaternion.hpp"
#include "helios/render/stroke_font.hpp"
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
constexpr std::array<double, 5> k_dial_ticks{0.0, 0.25, 0.5, 0.75, 1.0};
constexpr double k_plate_depth_m = 0.006;
constexpr double k_handle_reach_m = 0.03; // how far a lever's handle stands out of its slot
constexpr double k_label_gap_m = 0.035;
// The lettering on the panel (D38): painted labels, and readings lit on a dark window.
constexpr double k_letter_height_m = 0.012; // a capital's
constexpr double k_paint_m = 0.002;         // how far lettering stands out of what it is on
constexpr double k_window_depth_m = 0.004;
constexpr std::uint32_t k_reading_light = rgba(255, 190, 90);
constexpr std::uint32_t k_reading_window = rgba(12, 12, 14);

constexpr std::uint32_t k_dead = rgba(90, 90, 95);
constexpr double k_cover_stand_m = 0.03;                    // how far a guard's cover stands off the panel
constexpr double k_stick_lean_rad = std::numbers::pi / 9.0; // 20°: a stick's full deflection
constexpr double k_encoder_click_rad = std::numbers::pi / 6.0;
constexpr std::uint32_t k_hand = rgba(235, 190, 150);
constexpr double k_hand_size_m = 0.08;
// A controller's hand (D39): a flat hand open, a fist closed.
constexpr double k_hand_length_m = 0.17;
constexpr double k_hand_width_m = 0.085;
constexpr double k_hand_thickness_m = 0.03;
constexpr double k_fist_m = 0.09;

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

// The axes the pilot's look is given in, to the universe's: the vessel's, or the ground's for a
// pilot on foot outside.
[[nodiscard]] Matrix3 look_frame(const Placement& vessel, const sim::PilotView& pilot) noexcept {
    return pilot.ground.has_value() ? math::to_matrix(*pilot.ground) : vessel.rotation;
}

[[nodiscard]] Placement part_placement(const Placement& vessel, const sim::PartView& part) noexcept {
    return {.rotation = vessel.rotation * part.orientation, .origin_m = vessel.point(part.position_m)};
}

[[nodiscard]] Placement seat_placement(const Placement& part, const vessel::Cockpit& cockpit) noexcept {
    return {.rotation = part.rotation * cockpit.seat_to_part(), .origin_m = part.point(cockpit.eye_m)};
}

// A ladder on the hull (BRIEFING D35): two rails along its line and a rung every so often,
// standing a little off the hull. `seat` places the seat's axes the ladder is given in.
void add_ladder(FlightGeometry& geometry, const Placement& seat, const vessel::Ladder& ladder) {
    constexpr double k_rail_apart_m = 0.45;
    constexpr double k_rung_every_m = 0.3;
    constexpr double k_bar_m = 0.035;
    constexpr double k_off_hull_m = 0.06;
    constexpr std::uint32_t k_ladder_colour = rgba(200, 170, 60);
    const double length_m = math::norm(ladder.to_m - ladder.from_m);
    if (!(length_m > 0.0)) {
        return;
    }
    const Vector3 along = (ladder.to_m - ladder.from_m) / length_m;
    const Vector3 square = ladder.out_m - math::dot(ladder.out_m, along) * along;
    if (!(math::norm(square) > 1.0e-9)) {
        return;
    }
    const Vector3 out = square / math::norm(square);
    const Vector3 across = math::cross(out, along); // (along, across, out) is right-handed
    const Vector3 middle_m = 0.5 * (ladder.from_m + ladder.to_m) + k_off_hull_m * out;
    const auto bar = [&](const Vector3& centre_m, const Vector3& x, const Vector3& y, const Vector3& z) {
        geometry.solids.push_back(SolidInstance{
            .shape = SolidShape::Box,
            .model = model_of(seat.direction(x), seat.direction(y), seat.direction(z), seat.point(centre_m)),
            .abgr = k_ladder_colour,
            .cabin_light = k_outside_light});
    };
    for (const double side : {-0.5, 0.5}) {
        bar(middle_m + side * k_rail_apart_m * across, length_m * along, k_bar_m * across, k_bar_m * out);
    }
    const auto rungs = static_cast<int>(std::floor(length_m / k_rung_every_m));
    for (int rung = 0; rung <= rungs; ++rung) {
        const Vector3 centre_m = ladder.from_m + (rung * k_rung_every_m) * along + k_off_hull_m * out;
        bar(centre_m, k_bar_m * along, k_rail_apart_m * across, k_bar_m * out);
    }
}

// The suit's tether (BRIEFING D36): a self-reeling line, as on the space station's walks. A reel
// where it is made fast pays it out and takes it in, so it runs straight from the reel to the
// suit whatever its length, and never lies loose; round the hull it bends (D42).
void add_tether(FlightGeometry& geometry, const sim::VesselView& suit, const sim::VesselView& host,
                const sim::TetherView& tether, const Vector3& camera_m) {
    constexpr double k_thickness_m = 0.02;
    constexpr double k_reel_m = 0.12;
    constexpr std::uint32_t k_tether_colour = rgba(225, 215, 190);
    constexpr std::uint32_t k_reel_colour = rgba(70, 72, 80);
    const Placement host_frame = vessel_placement(host, camera_m);
    const Vector3 reel_m = host_frame.point(tether.point_m);
    geometry.solids.push_back(
        SolidInstance{.shape = SolidShape::Box,
                      .model = model_of(host_frame.direction({k_reel_m, 0.0, 0.0}),
                                        host_frame.direction({0.0, k_reel_m, 0.0}),
                                        host_frame.direction({0.0, 0.0, k_reel_m}), reel_m),
                      .abgr = k_reel_colour,
                      .cabin_light = k_outside_light});
    const auto add_piece = [&](const Vector3& from_m, const Vector3& to_m) {
        const Vector3 span_m = to_m - from_m;
        const double apart_m = math::norm(span_m);
        if (!(apart_m > 1.0e-3)) {
            return;
        }
        const Vector3 along = span_m / apart_m;
        const Vector3 any = std::abs(along.x) < 0.9 ? Vector3{1.0, 0.0, 0.0} : Vector3{0.0, 1.0, 0.0};
        const Vector3 across = unit_or(math::cross(along, any), Vector3{0.0, 0.0, 1.0});
        const Vector3 third = math::cross(along, across);
        geometry.solids.push_back(SolidInstance{
            .shape = SolidShape::Box,
            .model = model_of(span_m, k_thickness_m * across, k_thickness_m * third, from_m + 0.5 * span_m),
            .abgr = k_tether_colour,
            .cabin_light = k_outside_light});
    };
    Vector3 from_m = reel_m;
    for (const Vector3& bend_m : tether.bends_m) {
        const Vector3 to_m = host_frame.point(bend_m);
        add_piece(from_m, to_m);
        from_m = to_m;
    }
    add_piece(from_m, suit.position_m - camera_m);
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

// How far along a handle's travel `off_m` (camera space, from its centre) is, from -1 to 1.
[[nodiscard]] double side_of(const ControlHandle& handle, const Vector3& off_m) noexcept {
    const Vector3 travel{handle.travel.x, handle.travel.y, handle.travel.z};
    const double length_m = math::norm(travel);
    if (!(length_m > 0.0) || !(handle.radius_m > 0.0F)) {
        return 0.0;
    }
    return std::clamp(math::dot(off_m, travel) / (length_m * static_cast<double>(handle.radius_m)), -1.0,
                      1.0);
}

// Where a value stands in a signal's range, from 0 to 1.
[[nodiscard]] double fraction_of(const sim::SignalView& signal) noexcept {
    return signal.maximum > signal.minimum
               ? std::clamp((signal.value - signal.minimum) / (signal.maximum - signal.minimum), 0.0, 1.0)
               : 0.0;
}

// The axes a control or an indicator is drawn in, from its place: `out` of the panel, `up`
// as lettered, and `right` as seen by whoever faces it. (right, up, out) is right-handed.
struct PanelAxes {
    Vector3 out;
    Vector3 up;
    Vector3 right;
};

[[nodiscard]] PanelAxes axes_of(const vessel::PanelPlace& place) noexcept {
    return {.out = place.facing, .up = place.up, .right = math::cross(place.up, place.facing)};
}

[[nodiscard]] Matrix3 turn_about(const Vector3& unit_axis, double angle_rad) noexcept {
    return math::to_matrix(math::from_axis_angle(unit_axis, angle_rad));
}

// How the moving part of a control's model is moved (BRIEFING D44), in the seat's axes: turned
// about the model's pivot, then shifted.
struct Motion {
    Matrix3 turn;
    Vector3 shift_m;
};

// Builds the solids, the lettering and the handles of one cockpit's controls and indicators
// (BRIEFING D43), each in the built-in shape its look names.
class PanelBuilder {
public:
    PanelBuilder(FlightGeometry& geometry, const Placement& seat, const sim::VesselView& vessel,
                 const ModelLibrary* models) noexcept
        : geometry_(geometry), seat_(seat), vessel_(vessel), models_(models) {}

    void add(const vessel::Control& control, std::size_t index, bool guard_open) {
        using Type = vessel::Control::Type;
        const vessel::Look& look = control.look;
        const Model* model = begin(look);
        const PanelAxes axes = axes_of(control.place);
        const Vector3& at_m = control.place.position_m;
        ControlHandle handle{.control = index,
                             .guard = false,
                             .centre = to_float(seat_.point(at_m)),
                             .radius_m = static_cast<float>(0.6 * look.size_m),
                             .travel = {},
                             .travel_up = {},
                             .label = control.label,
                             .live = live(control)};
        double half_height_m = 0.5 * look.size_m;
        switch (control.type) {
        case Type::Momentary: momentary(control, axes, handle.live); break;
        case Type::Switch:    half_height_m = switch_control(control, axes, handle); break;
        case Type::Axis:      half_height_m = axis(control, axes, handle); break;
        case Type::Stick:     half_height_m = stick(control, axes, handle); break;
        case Type::Encoder:   encoder(control, axes, handle); break;
        }
        finish(model, look, control.place, handle.live);
        bool hidden = false;
        if (control.guard.has_value()) {
            // The cover: closed, it hides the control from the hand; open, it stands up off the
            // panel, hinged at its foot.
            const vessel::Look& cover = control.guard->look;
            const Model* cover_model = begin(cover);
            const double width_m = cover.size_m > 0.0 ? cover.size_m : 1.4 * look.size_m;
            const double height_m =
                cover.size_m > 0.0 ? cover.size_m : 2.0 * half_height_m + 0.3 * look.size_m;
            const double depth_m = 0.006;
            const double angle_rad = guard_open ? cover.travel : 0.0;
            const Vector3 lid_up = std::cos(angle_rad) * axes.up + std::sin(angle_rad) * axes.out;
            const Vector3 lid_out = std::cos(angle_rad) * axes.out - std::sin(angle_rad) * axes.up;
            const Vector3 hinge_m = at_m - (0.5 * height_m) * axes.up + k_cover_stand_m * axes.out;
            const Vector3 lid_m = hinge_m + (0.5 * height_m) * lid_up;
            solid(SolidShape::Box, lid_m, width_m * axes.right, height_m * lid_up, depth_m * lid_out,
                  box_colour(cover.colour));
            motion_.turn = turn_about(axes.right, angle_rad);
            finish(cover_model, cover, control.place, true);
            geometry_.get().controls.push_back(ControlHandle{.control = index,
                                                             .guard = true,
                                                             .centre = to_float(seat_.point(lid_m)),
                                                             .radius_m = static_cast<float>(0.5 * height_m),
                                                             .travel = {},
                                                             .travel_up = {},
                                                             .label = control.label,
                                                             .live = true});
            hidden = !guard_open;
            half_height_m = std::max(half_height_m, 0.5 * height_m);
        }
        label(control.label, at_m, axes, half_height_m, handle.live, look.paint);
        if (!hidden) {
            geometry_.get().controls.push_back(std::move(handle));
        }
    }

    void add(const vessel::Indicator& indicator) {
        using Type = vessel::Indicator::Type;
        const vessel::Look& look = indicator.look;
        const Model* model = begin(look);
        const PanelAxes axes = axes_of(indicator.place);
        const Vector3& at_m = indicator.place.position_m;
        const auto signal = find_signal(vessel_, indicator.signal);
        const bool is_live = signal.has_value();
        const double value = is_live ? signal->get().value : 0.0;
        const double size_m = look.size_m;
        double half_height_m = 0.5 * size_m;
        std::string reading;
        switch (indicator.type) {
        case Type::Gauge: {
            const double fraction =
                std::clamp((value - indicator.minimum) / (indicator.maximum - indicator.minimum), 0.0, 1.0);
            motion_.turn = turn_about(axes.out, -(fraction - 0.5) * look.travel);
            if (look.shape == "tape") {
                motion_ = Motion{.turn = {}, .shift_m = ((fraction - 0.5) * size_m) * axes.up};
                solid(SolidShape::Box, at_m, (0.3 * size_m) * axes.right, size_m * axes.up,
                      k_dial_depth_m * axes.out, box_colour(look.colour));
                for (const double tick : k_dial_ticks) {
                    solid(SolidShape::Box,
                          at_m + ((tick - 0.5) * size_m) * axes.up + k_dial_depth_m * axes.out,
                          (0.12 * size_m) * axes.right, 0.002 * axes.up, 0.004 * axes.out,
                          box_colour(look.paint));
                }
                if (is_live) {
                    solid(SolidShape::Box,
                          at_m + ((fraction - 0.5) * size_m) * axes.up + (k_dial_depth_m + 0.004) * axes.out,
                          (0.3 * size_m) * axes.right, 0.006 * axes.up, 0.006 * axes.out,
                          box_colour(look.accent));
                }
            } else {
                dial(at_m, axes, 0.5 * size_m, look,
                     is_live ? std::optional<double>{fraction} : std::nullopt);
            }
            if (is_live) {
                reading = format_reading(value * indicator.scale, indicator.unit);
            }
            break;
        }
        case Type::State: {
            const auto reached = static_cast<double>(std::ranges::count_if(
                indicator.thresholds, [&](double threshold) { return is_live && value >= threshold; }));
            const double lit = reached / static_cast<double>(indicator.thresholds.size());
            lit_ = lit;
            if (look.shape == "talkback") {
                solid(SolidShape::Cylinder, at_m, k_dial_depth_m * axes.out, (0.5 * size_m) * axes.right,
                      (0.5 * size_m) * axes.up, if_live(is_live, box_colour(look.colour)));
                if (reached > 0.0) {
                    // The barber pole: stripes across the window.
                    const Vector3 along = std::numbers::sqrt2 / 2.0 * (axes.right + axes.up);
                    const Vector3 across = std::numbers::sqrt2 / 2.0 * (axes.up - axes.right);
                    for (const double offset : {-0.25, 0.0, 0.25}) {
                        solid(SolidShape::Box,
                              at_m + (offset * size_m) * across + (k_dial_depth_m + 0.001) * axes.out,
                              (0.8 * size_m) * along, (0.08 * size_m) * across, 0.002 * axes.out,
                              box_colour(look.accent));
                    }
                }
            } else {
                const Vector3 colour = look.colour + lit * (look.accent - look.colour);
                solid(SolidShape::Box, at_m + 0.006 * axes.out, size_m * axes.right, size_m * axes.up,
                      0.012 * axes.out, if_live(is_live, box_colour(colour)));
            }
            break;
        }
        case Type::Readout: {
            reading = is_live ? format_reading(value * indicator.scale, indicator.unit) : std::string{"----"};
            solid(SolidShape::Box, at_m + (0.5 * k_window_depth_m) * axes.out, size_m * axes.right,
                  (2.2 * k_letter_height_m) * axes.up, k_window_depth_m * axes.out, box_colour(look.colour));
            lettering(reading, at_m + (k_window_depth_m + k_paint_m) * axes.out, axes.right, axes.up,
                      if_live(is_live, box_colour(look.accent)));
            reading.clear();
            half_height_m = 1.1 * k_letter_height_m;
            break;
        }
        }
        finish(model, look, indicator.place, is_live);
        label(indicator.label, at_m, axes, half_height_m, is_live, look.paint);
        if (!reading.empty()) {
            const Vector3 window_m = at_m - (half_height_m + k_label_gap_m) * axes.up;
            const double width_m = (static_cast<double>(text_width(reading)) + 1.0) * k_letter_height_m;
            solid(SolidShape::Box, window_m + (0.5 * k_window_depth_m) * axes.out, width_m * axes.right,
                  (2.0 * k_letter_height_m) * axes.up, k_window_depth_m * axes.out, k_reading_window);
            lettering(reading, window_m + (k_window_depth_m + k_paint_m) * axes.out, axes.right, axes.up,
                      k_reading_light);
        }
    }

private:
    // Whether the vessel has every signal the control moves.
    [[nodiscard]] bool live(const vessel::Control& control) const {
        const auto has = [&](const std::string& name) { return find_signal(vessel_, name).has_value(); };
        bool all =
            (control.signal.empty() || has(control.signal)) && std::ranges::all_of(control.signals, has);
        for (const vessel::SignalSetting& setting : control.press) {
            all = all && has(setting.signal);
        }
        for (const vessel::SwitchPosition& position : control.positions) {
            for (const vessel::SignalSetting& setting : position.settings) {
                all = all && has(setting.signal);
            }
        }
        return all;
    }

    void momentary(const vessel::Control& control, const PanelAxes& axes, bool is_live) {
        const vessel::Look& look = control.look;
        // Lit while everything it sets holds (a mode button), never for one that only steps.
        bool lit = is_live && !control.press.empty();
        for (const vessel::SignalSetting& setting : control.press) {
            const auto signal = find_signal(vessel_, setting.signal);
            lit = lit && signal.has_value() && signal->get().value == setting.value;
        }
        lit_ = lit ? 1.0 : 0.0;
        const std::uint32_t colour = if_live(is_live, box_colour(lit ? look.accent : look.colour));
        const Vector3& at_m = control.place.position_m;
        if (look.shape == "round_button") {
            solid(SolidShape::Cylinder, at_m + 0.01 * axes.out, 0.02 * axes.out,
                  (0.5 * look.size_m) * axes.right, (0.5 * look.size_m) * axes.up, colour);
        } else {
            solid(SolidShape::Box, at_m + 0.01 * axes.out, look.size_m * axes.right, look.size_m * axes.up,
                  0.02 * axes.out, colour);
        }
    }

    // Returns half the height it takes on the panel.
    double switch_control(const vessel::Control& control, const PanelAxes& axes, ControlHandle& handle) {
        const vessel::Look& look = control.look;
        const Vector3& at_m = control.place.position_m;
        const double size_m = look.size_m;
        const std::optional<std::size_t> position =
            vessel::switch_position(control, [&](const std::string& name) {
                const auto signal = find_signal(vessel_, name);
                return signal.has_value() ? std::optional<double>{signal->get().value} : std::nullopt;
            });
        const double fraction = position.has_value() ? static_cast<double>(*position)
                                                           / static_cast<double>(control.positions.size() - 1)
                                                     : 0.0;
        const auto steps = static_cast<double>(control.positions.size() - 1);
        const std::uint32_t accent = if_live(handle.live && position.has_value(), box_colour(look.accent));
        lit_ = fraction;
        if (look.shape == "rotary") {
            motion_.turn = turn_about(axes.out, -(fraction - 0.5) * look.travel);
            const double radius_m = 0.5 * size_m;
            solid(SolidShape::Cylinder, at_m + 0.01 * axes.out, 0.02 * axes.out, radius_m * axes.right,
                  radius_m * axes.up, box_colour(look.colour));
            const auto direction_at = [&](double at) {
                const double angle_rad = (at - 0.5) * look.travel;
                return std::cos(angle_rad) * axes.up + std::sin(angle_rad) * axes.right;
            };
            for (std::size_t mark = 0; mark < control.positions.size(); ++mark) {
                const Vector3 direction = direction_at(static_cast<double>(mark) / steps);
                solid(SolidShape::Box, at_m + (1.25 * radius_m) * direction + 0.002 * axes.out,
                      (0.06 * size_m) * math::cross(direction, axes.out), (0.2 * radius_m) * direction,
                      0.002 * axes.out, box_colour(look.paint));
            }
            const Vector3 pointer = direction_at(fraction);
            solid(SolidShape::Box, at_m + (0.5 * radius_m) * pointer + 0.022 * axes.out,
                  (0.15 * size_m) * math::cross(pointer, axes.out), (0.9 * radius_m) * pointer,
                  0.004 * axes.out, accent);
            handle.travel = to_float(seat_.direction(radius_m * axes.right));
            return 1.4 * radius_m;
        }
        if (look.shape == "breaker") {
            // Pulled out (its first position), a band shows; pushed in, it is flush.
            const double out_m = (1.0 - fraction) * look.travel + 0.004;
            motion_.shift_m = ((1.0 - fraction) * look.travel) * axes.out;
            solid(SolidShape::Cylinder, at_m + (0.5 * out_m) * axes.out, out_m * axes.out,
                  (0.5 * size_m) * axes.right, (0.5 * size_m) * axes.up,
                  fraction < 0.5 ? accent : box_colour(look.colour));
            handle.travel = to_float(seat_.direction(-look.travel * axes.out));
            return 0.5 * size_m;
        }
        // A toggle or a rocker on a plate, moving between its positions up the panel.
        solid(SolidShape::Box, at_m, size_m * axes.right, (1.6 * size_m) * axes.up,
              k_plate_depth_m * axes.out, box_colour(look.colour));
        const double angle_rad = (2.0 * fraction - 1.0) * look.travel;
        motion_.turn = turn_about(axes.right, -angle_rad);
        if (look.shape == "rocker") {
            const Vector3 face = std::cos(angle_rad) * axes.out + std::sin(angle_rad) * axes.up;
            const Vector3 along = std::cos(angle_rad) * axes.up - std::sin(angle_rad) * axes.out;
            solid(SolidShape::Box, at_m + 0.012 * axes.out, (0.9 * size_m) * axes.right,
                  (1.4 * size_m) * along, 0.014 * face,
                  fraction >= 0.5 ? accent : if_live(handle.live, box_colour(look.paint)));
        } else {
            const double length_m = 0.7 * size_m;
            const Vector3 lean = std::cos(angle_rad) * axes.out + std::sin(angle_rad) * axes.up;
            const Vector3 grip_m = at_m + (0.5 * length_m) * lean;
            solid(SolidShape::Box, grip_m, (0.22 * size_m) * axes.right, length_m * lean,
                  (0.22 * size_m) * math::cross(axes.right, lean), accent);
            handle.centre = to_float(seat_.point(at_m + (0.7 * length_m) * lean));
        }
        // From one position to the next, for a drag (and the side a click is on).
        handle.travel = to_float(seat_.direction((1.2 * size_m / std::max(steps, 1.0)) * axes.up));
        return 0.8 * size_m;
    }

    double axis(const vessel::Control& control, const PanelAxes& axes, ControlHandle& handle) {
        const vessel::Look& look = control.look;
        const Vector3& at_m = control.place.position_m;
        const auto signal = find_signal(vessel_, control.signal);
        const double fraction = signal.has_value() ? fraction_of(signal->get()) : 0.0;
        const std::uint32_t accent = if_live(handle.live, box_colour(look.accent));
        const double radius_m = 0.5 * look.size_m;
        if (look.shape == "knob" || look.shape == "wheel") {
            const bool wheel = look.shape == "wheel";
            // A knob faces out of the panel; a wheel stands on its edge, turning about `right`.
            const Vector3 axle = wheel ? axes.right : axes.out;
            const Vector3 first = wheel ? axes.up : axes.right;
            const Vector3 second = wheel ? axes.out : axes.up;
            solid(SolidShape::Cylinder, at_m + (wheel ? Vector3{} : 0.012 * axes.out),
                  (wheel ? 0.25 * look.size_m : 0.024) * axle, radius_m * first, radius_m * second,
                  box_colour(look.colour));
            const double angle_rad = (fraction - 0.5) * look.travel;
            motion_.turn = turn_about(wheel ? axes.right : axes.out, -angle_rad);
            const Vector3 mark = wheel ? std::cos(angle_rad) * axes.out + std::sin(angle_rad) * axes.up
                                       : std::cos(angle_rad) * axes.up + std::sin(angle_rad) * axes.right;
            solid(SolidShape::Box, at_m + (0.75 * radius_m) * mark + (wheel ? Vector3{} : 0.025 * axes.out),
                  (0.12 * look.size_m) * math::cross(mark, axle), (0.4 * radius_m) * mark,
                  (wheel ? 0.27 * look.size_m : 0.004) * axle, accent);
            // Turned by dragging across it: right turns it clockwise (a wheel: its top away).
            handle.travel = to_float(seat_.direction((2.0 * radius_m) * (wheel ? axes.up : axes.right)));
            handle.radius_m = static_cast<float>(radius_m);
            if (signal.has_value()) {
                handle.label = control.label;
            }
            return radius_m;
        }
        // A lever in a slot, sliding along `up`.
        motion_.shift_m = ((fraction - 0.5) * look.travel) * axes.up;
        solid(SolidShape::Box, at_m, 0.025 * axes.right, look.travel * axes.up, k_plate_depth_m * axes.out,
              box_colour(look.colour));
        const Vector3 grip_m =
            at_m + ((fraction - 0.5) * look.travel) * axes.up + k_handle_reach_m * axes.out;
        solid(SolidShape::Box, grip_m, look.size_m * axes.right, 0.03 * axes.up,
              (2.0 * k_handle_reach_m) * axes.out, accent);
        for (const double detent : control.detents) {
            solid(SolidShape::Box, at_m + ((detent - 0.5) * look.travel) * axes.up + 0.002 * axes.out,
                  0.045 * axes.right, 0.003 * axes.up, 0.004 * axes.out, box_colour(look.paint));
        }
        handle.centre = to_float(seat_.point(grip_m));
        handle.radius_m = 0.06F;
        handle.travel = to_float(seat_.direction(look.travel * axes.up));
        return 0.5 * look.travel;
    }

    double stick(const vessel::Control& control, const PanelAxes& axes, ControlHandle& handle) {
        const vessel::Look& look = control.look;
        const Vector3& at_m = control.place.position_m;
        std::array<double, 3> deflection{};
        for (std::size_t index = 0; index < control.signals.size() && index < deflection.size(); ++index) {
            if (const auto signal = find_signal(vessel_, control.signals[index]); signal.has_value()) {
                deflection.at(index) = 2.0 * fraction_of(signal->get()) - 1.0;
            }
        }
        solid(SolidShape::Box, at_m, (2.0 * look.size_m) * axes.right, (2.0 * look.size_m) * axes.up,
              k_plate_depth_m * axes.out, box_colour(look.colour));
        const double reach = std::tan(k_stick_lean_rad);
        const Vector3 along = unit_or(
            axes.out + (reach * deflection[0]) * axes.right + (reach * deflection[1]) * axes.up, axes.out);
        const Vector3 side = unit_or(math::cross(axes.up, along), axes.right);
        const Vector3 front = math::cross(along, side);
        const Vector3 top_m = at_m + look.travel * along;
        solid(SolidShape::Cylinder, at_m + (0.5 * look.travel) * along, look.travel * along, 0.01 * side,
              0.01 * front, box_colour(look.colour));
        // The grip, turned by the twist.
        const double twist_rad = deflection[2] * k_stick_lean_rad;
        const Vector3 grip_side = std::cos(twist_rad) * side + std::sin(twist_rad) * front;
        // Leant from `out` to `along`, the shortest way, then twisted about `along`.
        const Vector3 lean_axis = math::cross(axes.out, along);
        const double lean_rad = std::atan2(math::norm(lean_axis), math::dot(axes.out, along));
        motion_.turn =
            turn_about(along, twist_rad)
            * (lean_rad > 0.0 ? turn_about(lean_axis / math::norm(lean_axis), lean_rad) : Matrix3{});
        solid(SolidShape::Box, top_m, look.size_m * grip_side, 0.09 * along,
              look.size_m * math::cross(grip_side, along), if_live(handle.live, box_colour(look.accent)));
        handle.centre = to_float(seat_.point(top_m));
        handle.radius_m = static_cast<float>(look.size_m);
        // A full deflection each way: the grip moves this far.
        handle.travel = to_float(seat_.direction((reach * look.travel) * axes.right));
        handle.travel_up = to_float(seat_.direction((reach * look.travel) * axes.up));
        return look.travel * 0.5;
    }

    void encoder(const vessel::Control& control, const PanelAxes& axes, ControlHandle& handle) {
        const vessel::Look& look = control.look;
        const Vector3& at_m = control.place.position_m;
        const auto signal = find_signal(vessel_, control.signal);
        const double radius_m = 0.5 * look.size_m;
        solid(SolidShape::Cylinder, at_m + 0.012 * axes.out, 0.024 * axes.out, radius_m * axes.right,
              radius_m * axes.up, box_colour(look.colour));
        const double clicks = signal.has_value() ? signal->get().value / control.step : 0.0;
        const double angle_rad = clicks * k_encoder_click_rad;
        motion_.turn = turn_about(axes.out, -angle_rad);
        const Vector3 mark = std::cos(angle_rad) * axes.up + std::sin(angle_rad) * axes.right;
        solid(SolidShape::Box, at_m + (0.6 * radius_m) * mark + 0.025 * axes.out,
              (0.12 * look.size_m) * math::cross(mark, axes.out), (0.6 * radius_m) * mark, 0.004 * axes.out,
              if_live(handle.live, box_colour(look.accent)));
        handle.travel = to_float(seat_.direction(radius_m * axes.right));
    }

    // A round dial with its ticks, and a needle at `fraction` of its sweep when there is one.
    void dial(const Vector3& at_m, const PanelAxes& axes, double radius_m, const vessel::Look& look,
              std::optional<double> fraction) {
        solid(SolidShape::Cylinder, at_m, k_dial_depth_m * axes.out, radius_m * axes.right,
              radius_m * axes.up, box_colour(look.colour));
        const auto direction_at = [&](double at) {
            const double angle_rad = (at - 0.5) * look.travel;
            return std::cos(angle_rad) * axes.up + std::sin(angle_rad) * axes.right;
        };
        for (const double tick : k_dial_ticks) {
            const Vector3 direction = direction_at(tick);
            solid(SolidShape::Box, at_m + (0.88 * radius_m) * direction + k_dial_depth_m * axes.out,
                  (0.16 * radius_m) * direction, (0.05 * radius_m) * math::cross(axes.out, direction),
                  0.004 * axes.out, box_colour(look.paint));
        }
        if (fraction.has_value()) {
            const Vector3 direction = direction_at(*fraction);
            solid(SolidShape::Box, at_m + (0.38 * radius_m) * direction + (k_dial_depth_m + 0.004) * axes.out,
                  (0.8 * radius_m) * direction, (0.08 * radius_m) * math::cross(axes.out, direction),
                  0.006 * axes.out, box_colour(look.accent));
        }
    }

    // The label lettered above something `half_height_m` high, upright as it is.
    void label(std::string_view text, const Vector3& at_m, const PanelAxes& axes, double half_height_m,
               bool is_live, const Vector3& paint) {
        lettering(text, at_m + (half_height_m + k_label_gap_m) * axes.up + k_paint_m * axes.out, axes.right,
                  axes.up, if_live(is_live, box_colour(paint)));
    }

    // A line of lettering centred on a point of the panel, in the seat's axes.
    void lettering(std::string_view text, const Vector3& centre_m, const Vector3& right, const Vector3& up,
                   std::uint32_t abgr) {
        append_text(geometry_.get().text, text,
                    TextPlacement{.centre_m = seat_.point(centre_m),
                                  .right_m = seat_.direction(k_letter_height_m * right),
                                  .up_m = seat_.direction(k_letter_height_m * up)},
                    abgr);
    }

    // The model a look is drawn with, if the library has it; until `finish`, the built-in shape
    // is then not drawn, only its motion and its lamp worked out.
    [[nodiscard]] const Model* begin(const vessel::Look& look) noexcept {
        const Model* model = models_ != nullptr && !look.model.empty() ? models_->find(look.model) : nullptr;
        modelled_ = model != nullptr;
        motion_ = Motion{};
        lit_ = 0.0;
        return model;
    }

    // Places the model's pieces as `place` and the motion say, in the look's colours.
    void finish(const Model* model, const vessel::Look& look, const vessel::PanelPlace& place, bool is_live) {
        modelled_ = false;
        if (model == nullptr) {
            return;
        }
        const PanelAxes axes = axes_of(place);
        // Rationale: glTF's x, y and z (right, up, towards the viewer) are the panel's right,
        // up and out, so a model is made facing whoever uses it.
        const Matrix3 frame = math::transpose(with_rows(axes.right, axes.up, axes.out));
        const Vector3 pivot_m = place.position_m + frame * model->pivot_m;
        for (const ModelPiece& piece : model->pieces) {
            Matrix3 linear = frame;
            Vector3 origin_m = place.position_m;
            if (piece.moving) {
                linear = motion_.turn * frame;
                origin_m = pivot_m + motion_.turn * (place.position_m - pivot_m) + motion_.shift_m;
            }
            const Matrix3 to_camera = seat_.rotation * linear;
            geometry_.get().models.push_back(ModelInstance{
                .mesh = piece.mesh,
                .model = model_of(to_camera * Vector3{1.0, 0.0, 0.0}, to_camera * Vector3{0.0, 1.0, 0.0},
                                  to_camera * Vector3{0.0, 0.0, 1.0}, seat_.point(origin_m)),
                .abgr = piece_colour(piece, look, is_live),
                .cabin_light = k_cabin_light});
        }
    }

    [[nodiscard]] std::uint32_t piece_colour(const ModelPiece& piece, const vessel::Look& look,
                                             bool is_live) const noexcept {
        switch (piece.paint) {
        case ModelPaint::Own:    return box_colour(piece.colour);
        case ModelPaint::Colour: return box_colour(look.colour);
        case ModelPaint::Accent: return if_live(is_live, box_colour(look.accent));
        case ModelPaint::Paint:  return box_colour(look.paint);
        case ModelPaint::Lamp:
            return if_live(is_live, box_colour(look.colour + lit_ * (look.accent - look.colour)));
        }
        return box_colour(piece.colour);
    }

    // `x`, `y`, `z`: what the unit shape's edges become, in the seat's axes. They must be
    // right-handed (x × y along z), or the solid would be drawn inside out. Not drawn while
    // a model stands in for the shape.
    void solid(SolidShape shape, const Vector3& centre_m, const Vector3& x, const Vector3& y,
               const Vector3& z, std::uint32_t abgr) {
        if (modelled_) {
            return;
        }
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
    const ModelLibrary* models_;
    Motion motion_;
    double lit_ = 0.0; // how far a lamp of the model is lit, from 0 to 1
    bool modelled_ = false;
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

ViewPoint pilot_view_point(const sim::VesselView& vessel, const sim::PilotView& pilot,
                           const math::Quaternion& view) {
    const Placement frame = vessel_placement(vessel, Vector3{});
    const Matrix3 eyes = look_frame(frame, pilot) * math::to_matrix(math::normalized(view));
    const Vector3 forward = eyes * Vector3{1.0, 0.0, 0.0};
    const Vector3 left = eyes * Vector3{0.0, 1.0, 0.0};
    const Vector3 up = eyes * Vector3{0.0, 0.0, 1.0};
    return ViewPoint{.position_from_focus_m = frame.point(pilot.position_m) + sim::k_pilot_eye_height_m * up,
                     .universe_to_camera = with_rows(-1.0 * left, up, -1.0 * forward)};
}

namespace {

// A tracked pose about a frame whose axes (forward, left, up) `frame` takes to the universe's.
[[nodiscard]] ViewPoint tracked_view_point(const Placement& frame, const TrackedPose& tracked) noexcept {
    const Matrix3 axes = frame.rotation * math::to_matrix(math::normalized(tracked.orientation));
    const Vector3 forward = axes * Vector3{1.0, 0.0, 0.0};
    const Vector3 left = axes * Vector3{0.0, 1.0, 0.0};
    const Vector3 up = axes * Vector3{0.0, 0.0, 1.0};
    return ViewPoint{.position_from_focus_m = frame.point(tracked.position_m),
                     .universe_to_camera = with_rows(-1.0 * left, up, -1.0 * forward)};
}

} // namespace

std::optional<ViewPoint> seat_view_point(const sim::VesselView& vessel, const TrackedPose& tracked) {
    const auto part = seat_part(vessel);
    if (!part.has_value()) {
        return std::nullopt;
    }
    const sim::PartView& view = vessel.parts[*part];
    const std::optional<vessel::Cockpit>& cockpit = view.datasheet->cockpit;
    if (!cockpit.has_value()) {
        return std::nullopt;
    }
    return tracked_view_point(
        seat_placement(part_placement(vessel_placement(vessel, Vector3{}), view), *cockpit), tracked);
}

ViewPoint pilot_view_point(const sim::VesselView& vessel, const sim::PilotView& pilot,
                           const math::Quaternion& view, const TrackedPose& tracked) {
    const Placement frame = vessel_placement(vessel, Vector3{});
    const Matrix3 body = look_frame(frame, pilot) * math::to_matrix(math::normalized(view));
    const Vector3 up = body * Vector3{0.0, 0.0, 1.0};
    return tracked_view_point(
        Placement{.rotation = body,
                  .origin_m = frame.point(pilot.position_m) + sim::k_pilot_eye_height_m * up},
        tracked);
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
                                     const ViewPoint& view, bool from_seat, const ModelLibrary* models) {
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
                    if (box.glass) {
                        continue;
                    }
                    geometry.solids.push_back(
                        SolidInstance{.shape = SolidShape::Box,
                                      .model = model_of(seat_frame.direction({box.size_m.x, 0.0, 0.0}),
                                                        seat_frame.direction({0.0, box.size_m.y, 0.0}),
                                                        seat_frame.direction({0.0, 0.0, box.size_m.z}),
                                                        seat_frame.point(box.centre_m)),
                                      .abgr = box_colour(box.colour),
                                      .cabin_light = k_cabin_light});
                }
                PanelBuilder panel(geometry, seat_frame, drawn, models);
                for (std::size_t control = 0; control < cockpit.controls.size(); ++control) {
                    const std::string& id = cockpit.controls[control].id;
                    panel.add(cockpit.controls[control], control,
                              std::ranges::find(part_view.open_guards, id) != part_view.open_guards.end());
                }
                for (const vessel::Indicator& indicator : cockpit.indicators) {
                    panel.add(indicator);
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
            if (interior.has_value() && interior->ladder.has_value()) {
                add_ladder(geometry, seat_placement(part_frame, *interior), *interior->ladder);
            }
        }
    }
    const std::optional<sim::PilotView>& pilot = snapshot.pilot;
    if (pilot.has_value() && pilot->tether.has_value() && pilot->vessel.index < snapshot.vessels.size()
        && pilot->tether->vessel.index < snapshot.vessels.size()) {
        add_tether(geometry, snapshot.vessels[pilot->vessel.index],
                   snapshot.vessels[pilot->tether->vessel.index], *pilot->tether, camera_m);
    }
    // What is loose in the cabin, where it is: a hand holds an item where it took hold of it.
    if (from_seat && pilot.has_value() && pilot->vessel.index == vessel) {
        const Placement frame = vessel_placement(flown, camera_m);
        for (const sim::ItemView& item : snapshot.items) {
            const Placement placed{.rotation = frame.rotation * math::to_matrix(item.orientation),
                                   .origin_m = frame.point(item.position_m)};
            geometry.solids.push_back(
                SolidInstance{.shape = SolidShape::Box,
                              .model = model_of(placed.direction({item.size_m.x, 0.0, 0.0}),
                                                placed.direction({0.0, item.size_m.y, 0.0}),
                                                placed.direction({0.0, 0.0, item.size_m.z}), placed.origin_m),
                              .abgr = box_colour(item.colour),
                              .cabin_light = k_cabin_light});
        }
    }
    // The hand that holds on, as a small block where it holds.
    if (from_seat && pilot.has_value() && pilot->grabbing && pilot->vessel.index == vessel) {
        const Placement frame = vessel_placement(flown, camera_m);
        geometry.solids.push_back(SolidInstance{.shape = SolidShape::Box,
                                                .model = model_of(frame.direction({k_hand_size_m, 0.0, 0.0}),
                                                                  frame.direction({0.0, k_hand_size_m, 0.0}),
                                                                  frame.direction({0.0, 0.0, k_hand_size_m}),
                                                                  frame.point(pilot->grip_m)),
                                                .abgr = k_hand,
                                                .cabin_light = k_cabin_light});
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

std::optional<ControlPick> pick_control(std::span<const ControlHandle> controls,
                                        const Float3& ray_direction) noexcept {
    std::optional<ControlPick> nearest;
    float nearest_m = 0.0F;
    const Vector3 ray{ray_direction.x, ray_direction.y, ray_direction.z};
    for (std::size_t index = 0; index < controls.size(); ++index) {
        const ControlHandle& handle = controls[index];
        if (!handle.live) {
            continue;
        }
        const Vector3 centre{handle.centre.x, handle.centre.y, handle.centre.z};
        const double along_m = math::dot(centre, ray);
        if (along_m <= 0.0) {
            continue; // behind the camera
        }
        const Vector3 off_m = along_m * ray - centre; // from the centre to the nearest point of the ray
        if (math::norm(off_m) <= static_cast<double>(handle.radius_m)
            && (!nearest.has_value() || along_m < static_cast<double>(nearest_m))) {
            nearest = ControlPick{.handle = index, .side = side_of(handle, off_m)};
            nearest_m = static_cast<float>(along_m);
        }
    }
    return nearest;
}

double travel_fraction(const Float3& centre, const Float3& travel, const Matrix4f& view_projection,
                       float width_px, float height_px, float dx_px, float dy_px) noexcept {
    const Float3 low{centre.x - (0.5F * travel.x), centre.y - (0.5F * travel.y),
                     centre.z - (0.5F * travel.z)};
    const Float3 high{low.x + travel.x, low.y + travel.y, low.z + travel.z};
    const auto from = project_to_screen(low, view_projection, width_px, height_px);
    const auto to = project_to_screen(high, view_projection, width_px, height_px);
    if (!from.has_value() || !to.has_value()) {
        return 0.0;
    }
    const double along_x = to->x - from->x;
    const double along_y = to->y - from->y;
    const double length_squared = (along_x * along_x) + (along_y * along_y);
    // Rationale: a travel seen end-on has no direction on the screen to drag it along.
    if (length_squared < 1.0) {
        return 0.0;
    }
    return ((dx_px * along_x) + (dy_px * along_y)) / length_squared;
}

std::optional<ControlPick> touch_control(std::span<const ControlHandle> controls, const Vector3& point_m,
                                         double reach_m) noexcept {
    std::optional<ControlPick> nearest;
    double nearest_m = 0.0;
    for (std::size_t index = 0; index < controls.size(); ++index) {
        const ControlHandle& handle = controls[index];
        if (!handle.live) {
            continue;
        }
        const Vector3 centre{handle.centre.x, handle.centre.y, handle.centre.z};
        const double off_m = math::norm(point_m - centre) - static_cast<double>(handle.radius_m);
        if (off_m <= reach_m && (!nearest.has_value() || off_m < nearest_m)) {
            nearest = ControlPick{.handle = index, .side = side_of(handle, point_m - centre)};
            nearest_m = off_m;
        }
    }
    return nearest;
}

double travel_fraction(const Float3& travel, const Vector3& moved_m) noexcept {
    const Vector3 along{travel.x, travel.y, travel.z};
    const double length_squared = math::dot(along, along);
    return length_squared > 1e-12 ? math::dot(moved_m, along) / length_squared : 0.0;
}

void add_controller_hands(FlightGeometry& geometry, std::span<const ControllerHand> hands) {
    for (const ControllerHand& hand : hands) {
        // Open, a flat hand reaching forward; closed, a fist. Its fingertips are at the front.
        const double length_m = hand.closed ? k_fist_m : k_hand_length_m;
        const double thick_m = hand.closed ? k_fist_m : k_hand_thickness_m;
        const Vector3 forward = hand.hand_to_space * Vector3{1.0, 0.0, 0.0};
        const Vector3 left = hand.hand_to_space * Vector3{0.0, 1.0, 0.0};
        const Vector3 up = hand.hand_to_space * Vector3{0.0, 0.0, 1.0};
        geometry.solids.push_back(
            SolidInstance{.shape = SolidShape::Box,
                          .model = model_of(length_m * forward, k_hand_width_m * left, thick_m * up,
                                            hand.position_m - (0.5 * length_m) * forward),
                          .abgr = k_hand,
                          .cabin_light = k_cabin_light});
    }
}

} // namespace helios::render
