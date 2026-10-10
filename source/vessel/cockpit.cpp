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

[[nodiscard]] core::Result<std::vector<InstrumentCommand>> parse_commands(const TomlValue& table) {
    const auto rows = table.array_or_empty("commands");
    if (!rows) {
        return std::unexpected(rows.error());
    }
    std::vector<InstrumentCommand> commands;
    for (const TomlValue& row : *rows) {
        auto signal = row.string_at("signal");
        const auto value = row.number_at("value");
        if (const auto error = first_error(row.expect_keys({"signal", "value"}), signal, value)) {
            return std::unexpected(*error);
        }
        commands.push_back(InstrumentCommand{.signal = std::move(*signal), .value = *value});
    }
    return commands;
}

[[nodiscard]] core::Result<Instrument> parse_instrument(const TomlValue& table) {
    using Kind = Instrument::Kind;
    constexpr std::array<std::pair<std::string_view, Kind>, 4> k_kinds{
        {{"lever", Kind::Lever}, {"switch", Kind::Switch}, {"button", Kind::Button}, {"dial", Kind::Dial}}};
    const Instrument defaults;
    const auto kind_name = table.string_at("kind");
    auto label = table.string_or("label", "");
    auto signal = table.string_or("signal", "");
    const auto position = vector_or(table, "position_m", Vector3{});
    const auto facing = vector_or(table, "facing", defaults.facing);
    const auto up = vector_or(table, "up", defaults.up);
    const auto size = table.number_or("size_m", defaults.size_m);
    const auto minimum = table.number_or("minimum", defaults.minimum);
    const auto maximum = table.number_or("maximum", defaults.maximum);
    const auto scale = table.number_or("scale", defaults.display_scale);
    auto unit = table.string_or("unit", "");
    auto commands = parse_commands(table);
    const auto step = table.number_or("step", 0.0);
    if (const auto error =
            first_error(table.expect_keys({"kind", "label", "signal", "position_m", "facing", "up", "size_m",
                                           "minimum", "maximum", "scale", "unit", "commands", "step"}),
                        kind_name, label, signal, position, facing, up, size, minimum, maximum, scale, unit,
                        commands, step)) {
        return std::unexpected(*error);
    }
    std::optional<Kind> kind;
    for (const auto& [name, value] : k_kinds) {
        if (name == *kind_name) {
            kind = value;
        }
    }
    if (!kind.has_value()) {
        return invalid(table, std::format("unknown instrument kind '{}' (known: lever, switch, button, dial)",
                                          *kind_name));
    }
    const auto axes = perpendicular_axes(table, *facing, *up, "an instrument's facing and up");
    if (!axes) {
        return std::unexpected(axes.error());
    }
    if (!is_finite(*position) || !std::isfinite(*size) || !(*size > 0.0)) {
        return invalid(table, "an instrument needs a position and a positive size_m");
    }
    if (*kind == Kind::Button) {
        const bool steps = !signal->empty() && std::isfinite(*step) && *step != 0.0;
        if (commands->empty() && !steps) {
            return invalid(table, "a button needs commands, or a signal and a step");
        }
    } else if (signal->empty()) {
        return invalid(table, "a lever, a switch or a dial needs a signal");
    }
    if (*kind == Kind::Dial
        && (!std::isfinite(*minimum) || !std::isfinite(*maximum) || !(*minimum < *maximum)
            || !std::isfinite(*scale))) {
        return invalid(table, "a dial needs minimum < maximum and a finite scale");
    }
    return Instrument{.kind = *kind,
                      .label = std::move(*label),
                      .signal = std::move(*signal),
                      .position_m = *position,
                      .facing = axes->first,
                      .up = axes->second,
                      .size_m = *size,
                      .minimum = *minimum,
                      .maximum = *maximum,
                      .display_scale = *scale,
                      .display_unit = std::move(*unit),
                      .commands = std::move(*commands),
                      .step = *step};
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
    const auto instruments = table.array_or_empty("instrument");
    const auto items = table.array_or_empty("item");
    if (const auto error = first_error(table.expect_keys({"eye_m", "forward", "up", "walkable", "boxes",
                                                          "instrument", "item", "airlock"}),
                                       eye, forward, up, walkable, boxes, instruments, items)) {
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
                    .instruments = {},
                    .items = {},
                    .airlock = {}};
    for (const TomlValue& row : *boxes) {
        auto box = parse_box(row);
        if (!box) {
            return std::unexpected(box.error());
        }
        cockpit.boxes.push_back(*box);
    }
    for (const TomlValue& row : *instruments) {
        auto instrument = parse_instrument(row);
        if (!instrument) {
            return std::unexpected(instrument.error());
        }
        cockpit.instruments.push_back(std::move(*instrument));
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
    return cockpit;
}

} // namespace helios::vessel
