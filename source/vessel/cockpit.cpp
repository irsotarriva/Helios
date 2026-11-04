#include "helios/vessel/cockpit.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <optional>
#include <string_view>
#include <utility>

#include "toml_vector.hpp"

namespace helios::vessel {

namespace {

using core::ErrorCode;
using core::first_error;
using core::TomlValue;
using detail::vector_or;
using math::Vector3;

[[nodiscard]] bool is_finite(const Vector3& vector) noexcept {
    return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

[[nodiscard]] std::unexpected<core::Error> invalid(const TomlValue& table, std::string_view message) {
    return core::fail(ErrorCode::ParseFailure, std::format("line {}: {}", table.line, message));
}

struct Axes {
    Vector3 first;  // unit
    Vector3 second; // unit, at right angles to `first`
};

// `first` normalised, and `second` with its part along `first` removed. Fails if either has no
// direction left.
[[nodiscard]] core::Result<Axes> perpendicular_axes(const TomlValue& table, const Vector3& first,
                                                    const Vector3& second, std::string_view names) {
    constexpr double k_least_norm = 1e-6;
    const double first_norm = math::norm(first);
    if (!is_finite(first) || !is_finite(second) || !(first_norm > k_least_norm)) {
        return invalid(table, std::format("{} must be finite directions", names));
    }
    const Vector3 unit_first = first / first_norm;
    const Vector3 across = second - math::dot(second, unit_first) * unit_first;
    const double across_norm = math::norm(across);
    if (!(across_norm > k_least_norm * std::max(1.0, math::norm(second)))) {
        return invalid(table, std::format("{} must not be parallel", names));
    }
    return Axes{.first = unit_first, .second = across / across_norm};
}

[[nodiscard]] core::Result<CockpitBox> parse_box(const TomlValue& table) {
    const auto centre = vector_or(table, "centre_m", Vector3{});
    const auto size = vector_or(table, "size_m", Vector3{});
    const auto colour = vector_or(table, "colour", CockpitBox{}.colour);
    const auto glass = table.boolean_or("glass", false);
    if (const auto error = first_error(table.expect_keys({"centre_m", "size_m", "colour", "glass"}), centre,
                                       size, colour, glass)) {
        return std::unexpected(*error);
    }
    if (!is_finite(*centre) || !(size->x > 0.0) || !(size->y > 0.0) || !(size->z > 0.0)
        || !is_finite(*size)) {
        return invalid(table, "a box needs a centre and a positive size_m");
    }
    for (const double channel : {colour->x, colour->y, colour->z}) {
        if (!(channel >= 0.0) || !(channel <= 1.0)) {
            return invalid(table, "a colour is three numbers from 0 to 1");
        }
    }
    return CockpitBox{.centre_m = *centre, .size_m = *size, .colour = *colour, .glass = *glass};
}

[[nodiscard]] core::Result<CockpitItem> parse_item(const TomlValue& table) {
    const CockpitItem defaults;
    auto name = table.string_at("name");
    const auto position = vector_or(table, "position_m", Vector3{});
    const auto size = vector_or(table, "size_m", Vector3{});
    const auto mass = table.number_or("mass_kg", defaults.mass_kg);
    const auto colour = vector_or(table, "colour", defaults.colour);
    if (const auto error =
            first_error(table.expect_keys({"name", "position_m", "size_m", "mass_kg", "colour"}), name,
                        position, size, mass, colour)) {
        return std::unexpected(*error);
    }
    if (name->empty() || !is_finite(*position) || !is_finite(*size) || !(size->x > 0.0) || !(size->y > 0.0)
        || !(size->z > 0.0) || !std::isfinite(*mass) || !(*mass > 0.0)) {
        return invalid(table, "an item needs a name, a position, a positive size_m and a positive mass_kg");
    }
    for (const double channel : {colour->x, colour->y, colour->z}) {
        if (!(channel >= 0.0) || !(channel <= 1.0)) {
            return invalid(table, "a colour is three numbers from 0 to 1");
        }
    }
    return CockpitItem{.name = std::move(*name),
                       .position_m = *position,
                       .size_m = *size,
                       .mass_kg = *mass,
                       .colour = *colour};
}

[[nodiscard]] core::Result<Airlock> parse_airlock(const TomlValue& table) {
    const Airlock defaults;
    const auto inside = vector_or(table, "inside_m", Vector3{});
    const auto outside = vector_or(table, "outside_m", Vector3{});
    const auto cycle = table.number_or("cycle_s", defaults.cycle_s);
    if (const auto error =
            first_error(table.expect_keys({"inside_m", "outside_m", "cycle_s"}), inside, outside, cycle)) {
        return std::unexpected(*error);
    }
    if (!is_finite(*inside) || !is_finite(*outside) || !(math::norm(*outside - *inside) > 0.0)
        || !std::isfinite(*cycle) || *cycle < 0.0) {
        return invalid(table,
                       "an airlock needs an inside_m, a different outside_m and a cycle_s of 0 or more");
    }
    return Airlock{.inside_m = *inside, .outside_m = *outside, .cycle_s = *cycle};
}

[[nodiscard]] core::Result<Ladder> parse_ladder(const TomlValue& table) {
    const Ladder defaults;
    const auto from = vector_or(table, "from_m", Vector3{});
    const auto to = vector_or(table, "to_m", Vector3{});
    const auto out = vector_or(table, "out_m", Vector3{});
    const auto side = table.number_or("side_m", defaults.side_m);
    if (const auto error =
            first_error(table.expect_keys({"from_m", "to_m", "out_m", "side_m"}), from, to, out, side)) {
        return std::unexpected(*error);
    }
    const double out_norm = math::norm(*out);
    if (!is_finite(*from) || !is_finite(*to) || !(math::norm(*to - *from) > 0.0) || !is_finite(*out)
        || !(out_norm > 0.0) || !std::isfinite(*side) || *side < 0.0) {
        return invalid(table, "a ladder needs a from_m, a different to_m, an out_m with a direction and a "
                              "side_m of 0 or more");
    }
    return Ladder{.from_m = *from, .to_m = *to, .out_m = *out / out_norm, .side_m = *side};
}

} // namespace

math::Matrix3 Cockpit::seat_to_part() const noexcept {
    const Vector3 left = math::cross(up, forward);
    return math::Matrix3{{forward.x, left.x, up.x, forward.y, left.y, up.y, forward.z, left.z, up.z}};
}

core::Result<Cockpit> parse_cockpit(const TomlValue& table) {
    const Cockpit defaults;
    const auto eye = vector_or(table, "eye_m", Vector3{});
    const auto forward = vector_or(table, "forward", defaults.forward);
    const auto up = vector_or(table, "up", defaults.up);
    const auto walkable = table.boolean_or("walkable", false);
    const auto boxes = table.array_or_empty("boxes");
    auto style = table.string_or("style", "");
    const auto controls = table.array_or_empty("control");
    const auto indicators = table.array_or_empty("indicator");
    const auto items = table.array_or_empty("item");
    if (const auto error =
            first_error(table.expect_keys({"eye_m", "forward", "up", "walkable", "boxes", "style", "control",
                                           "indicator", "item", "airlock", "ladder"}),
                        eye, forward, up, walkable, boxes, style, controls, indicators, items)) {
        return std::unexpected(*error);
    }
    const auto axes = perpendicular_axes(table, *forward, *up, "a cockpit's forward and up");
    if (!axes) {
        return std::unexpected(axes.error());
    }
    if (!is_finite(*eye)) {
        return invalid(table, "eye_m must be finite");
    }
    Cockpit cockpit{.eye_m = *eye,
                    .forward = axes->first,
                    .up = axes->second,
                    .walkable = *walkable,
                    .boxes = {},
                    .style = std::move(*style),
                    .controls = {},
                    .indicators = {},
                    .items = {},
                    .airlock = {}};
    for (const TomlValue& row : *boxes) {
        auto box = parse_box(row);
        if (!box) {
            return std::unexpected(box.error());
        }
        cockpit.boxes.push_back(*box);
    }
    // Controls and indicators share their ids: a binding or a saved guard names one of them.
    const auto taken = [&](const std::string& id) {
        return std::ranges::find(cockpit.controls, id, &Control::id) != cockpit.controls.end()
               || std::ranges::find(cockpit.indicators, id, &Indicator::id) != cockpit.indicators.end();
    };
    for (const TomlValue& row : *controls) {
        auto control = parse_control(row);
        if (!control) {
            return std::unexpected(control.error());
        }
        if (taken(control->id)) {
            return invalid(row, std::format("two controls or indicators are called '{}'", control->id));
        }
        cockpit.controls.push_back(std::move(*control));
    }
    for (const TomlValue& row : *indicators) {
        auto indicator = parse_indicator(row);
        if (!indicator) {
            return std::unexpected(indicator.error());
        }
        if (taken(indicator->id)) {
            return invalid(row, std::format("two controls or indicators are called '{}'", indicator->id));
        }
        cockpit.indicators.push_back(std::move(*indicator));
    }
    for (const TomlValue& row : *items) {
        auto item = parse_item(row);
        if (!item) {
            return std::unexpected(item.error());
        }
        if (std::ranges::find(cockpit.items, item->name, &CockpitItem::name) != cockpit.items.end()) {
            return invalid(row, std::format("two items are called '{}'", item->name));
        }
        cockpit.items.push_back(std::move(*item));
    }
    if (!cockpit.items.empty() && !cockpit.walkable) {
        return invalid(table, "items need a cabin: a cockpit with walkable = true");
    }
    if (table.contains("airlock")) {
        const auto airlock = table.at("airlock").and_then(parse_airlock);
        if (!airlock) {
            return std::unexpected(airlock.error());
        }
        if (!cockpit.walkable) {
            return invalid(table, "an airlock needs a cabin: a cockpit with walkable = true");
        }
        cockpit.airlock = *airlock;
    }
    if (table.contains("ladder")) {
        const auto ladder = table.at("ladder").and_then(parse_ladder);
        if (!ladder) {
            return std::unexpected(ladder.error());
        }
        if (!cockpit.walkable) {
            return invalid(table, "a ladder needs a cabin: a cockpit with walkable = true");
        }
        cockpit.ladder = *ladder;
    }
    return cockpit;
}

core::VoidResult resolve_looks(Cockpit& cockpit, std::span<const StyleSheet> sheets) {
    const auto named = [](const std::string& id) {
        return [&id](core::Error error) {
            error.context = std::format("'{}': {}", id, error.context);
            return error;
        };
    };
    for (Control& control : cockpit.controls) {
        auto look =
            resolve_look(sheets, cockpit.style, element_name(control.type), control.classes, control.own_look)
                .transform_error(named(control.id));
        if (!look) {
            return std::unexpected(look.error());
        }
        control.look = std::move(*look);
        if (control.guard.has_value()) {
            auto guard = resolve_look(sheets, cockpit.style, k_guard_element, control.classes, LookPatch{})
                             .transform_error(named(control.id));
            if (!guard) {
                return std::unexpected(guard.error());
            }
            control.guard->look = std::move(*guard);
        }
    }
    for (Indicator& indicator : cockpit.indicators) {
        auto look = resolve_look(sheets, cockpit.style, element_name(indicator.type), indicator.classes,
                                 indicator.own_look)
                        .transform_error(named(indicator.id));
        if (!look) {
            return std::unexpected(look.error());
        }
        indicator.look = std::move(*look);
    }
    return {};
}

} // namespace helios::vessel
