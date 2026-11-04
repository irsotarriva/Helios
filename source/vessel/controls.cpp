#include "helios/vessel/controls.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <utility>

#include "toml_vector.hpp"

namespace helios::vessel {

namespace {

using core::ErrorCode;
using core::first_error;
using core::TomlValue;
using detail::vector_or;
using math::Vector3;

[[nodiscard]] std::unexpected<core::Error> invalid(const TomlValue& table, std::string_view message) {
    return core::fail(ErrorCode::ParseFailure, std::format("line {}: {}", table.line, message));
}

[[nodiscard]] bool is_finite(const Vector3& vector) noexcept {
    return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

[[nodiscard]] bool is_colour(const Vector3& colour) noexcept {
    return std::ranges::all_of(std::array{colour.x, colour.y, colour.z},
                               [](double channel) { return channel >= 0.0 && channel <= 1.0; });
}

// The built-in look of each shape, and which shapes each element may have (the first is the
// element's own when nothing says otherwise). Rationale: the measures are those of real
// panels: a toggle's bat handle throws 25° each way, a dial sweeps 270°.
struct BuiltInShape {
    std::string_view element;
    std::string_view shape;
    double size_m;
    double travel;
    Vector3 colour;
    Vector3 accent;
};

constexpr Vector3 k_panel_dark{0.11, 0.12, 0.14};
constexpr Vector3 k_metal{0.82, 0.82, 0.80};
constexpr Vector3 k_amber{1.0, 0.75, 0.35};

constexpr std::array<BuiltInShape, 17> k_shapes{{
    {"momentary", "pushbutton", 0.05, 0.004, {0.67, 0.47, 0.16}, {1.0, 0.84, 0.43}},
    {"momentary", "round_button", 0.04, 0.004, {0.67, 0.47, 0.16}, {1.0, 0.84, 0.43}},
    {"switch", "toggle", 0.04, 0.44, k_panel_dark, k_metal},
    {"switch", "rocker", 0.04, 0.26, k_panel_dark, {0.31, 0.86, 0.43}},
    {"switch", "rotary", 0.045, 1.57, k_panel_dark, k_metal},
    {"switch", "breaker", 0.02, 0.008, k_panel_dark, {0.85, 0.85, 0.85}},
    {"axis", "lever", 0.07, 0.3, k_panel_dark, {0.8, 0.24, 0.2}},
    {"axis", "knob", 0.05, 4.71, k_panel_dark, k_metal},
    {"axis", "wheel", 0.12, 3.14, k_panel_dark, k_metal},
    {"stick", "stick", 0.045, 0.22, k_panel_dark, {0.25, 0.25, 0.27}},
    {"encoder", "knob", 0.04, 0.0, k_panel_dark, k_metal},
    {"guard", "cover", 0.0, 1.9, {0.75, 0.15, 0.12}, {0.75, 0.15, 0.12}},
    {"gauge", "dial", 0.11, 4.71, k_panel_dark, {1.0, 0.59, 0.16}},
    {"gauge", "tape", 0.12, 0.0, k_panel_dark, {1.0, 0.59, 0.16}},
    {"state", "lamp", 0.03, 0.0, {0.22, 0.2, 0.16}, k_amber},
    {"state", "talkback", 0.03, 0.0, {0.45, 0.45, 0.47}, {0.95, 0.95, 0.95}},
    {"readout", "digits", 0.1, 0.0, {0.05, 0.05, 0.06}, k_amber},
}};

constexpr std::array<std::string_view, 9> k_elements{"momentary", "switch", "axis",  "stick",  "encoder",
                                                     "guard",     "gauge",  "state", "readout"};

[[nodiscard]] std::string shapes_of(std::string_view element) {
    std::string names;
    for (const BuiltInShape& shape : k_shapes) {
        if (shape.element == element) {
            names += names.empty() ? std::string{shape.shape} : std::format(", {}", shape.shape);
        }
    }
    return names;
}

[[nodiscard]] core::Result<LookPatch> parse_look_patch(const TomlValue& table) {
    LookPatch patch;
    for (std::size_t index = 0; index < table.keys.size(); ++index) {
        const std::string& key = table.keys[index];
        const TomlValue& value = table.items[index];
        if (key == "shape") {
            if (value.kind != TomlValue::Kind::String) {
                return invalid(table, "a look's shape is a name");
            }
            patch.shape = value.text;
        } else if (key == "size_m" || key == "travel") {
            if (value.kind != TomlValue::Kind::Number || !std::isfinite(value.number) || value.number < 0.0) {
                return invalid(table, std::format("a look's {} is a number, 0 or more", key));
            }
            (key == "size_m" ? patch.size_m : patch.travel) = value.number;
        } else if (key == "colour" || key == "accent" || key == "paint") {
            const auto colour = vector_or(table, key, Vector3{});
            if (!colour) {
                return std::unexpected(colour.error());
            }
            if (!is_colour(*colour)) {
                return invalid(table, "a colour is three numbers from 0 to 1");
            }
            if (key == "colour") {
                patch.colour = *colour;
            } else if (key == "accent") {
                patch.accent = *colour;
            } else {
                patch.paint = *colour;
            }
        } else if (key == "model") {
            return invalid(table, "a look's model (a glTF file) is not read yet: built-in shapes only");
        } else {
            return invalid(table, std::format("unknown key '{}' in a look (known: shape, size_m, travel, "
                                              "colour, accent, paint)",
                                              key));
        }
    }
    return patch;
}

[[nodiscard]] core::Result<LookPatch> look_or_empty(const TomlValue& table) {
    if (!table.contains("look")) {
        return LookPatch{};
    }
    return table.at("look").and_then([](const TomlValue& look) -> core::Result<LookPatch> {
        if (look.kind != TomlValue::Kind::Table) {
            return invalid(look, "look is a table");
        }
        return parse_look_patch(look);
    });
}

// `class = "red large"`: names separated by spaces.
[[nodiscard]] core::Result<std::vector<std::string>> classes_of(const TomlValue& table) {
    return table.string_or("class", "").transform([](const std::string& text) {
        std::vector<std::string> classes;
        std::size_t start = 0;
        while (start < text.size()) {
            const std::size_t end = std::min(text.find(' ', start), text.size());
            if (end > start) {
                classes.push_back(text.substr(start, end - start));
            }
            start = end + 1;
        }
        return classes;
    });
}

[[nodiscard]] core::Result<PanelPlace> parse_place(const TomlValue& table) {
    const PanelPlace defaults;
    const auto position = vector_or(table, "position_m", Vector3{});
    const auto facing = vector_or(table, "facing", defaults.facing);
    const auto up = vector_or(table, "up", defaults.up);
    if (const auto error = first_error(position, facing, up)) {
        return std::unexpected(*error);
    }
    const double facing_norm = math::norm(*facing);
    if (!is_finite(*position) || !is_finite(*facing) || !is_finite(*up) || !(facing_norm > 1e-6)) {
        return invalid(table, "a position_m and a facing with a direction are needed");
    }
    const Vector3 out = *facing / facing_norm;
    const Vector3 across = *up - math::dot(*up, out) * out;
    const double across_norm = math::norm(across);
    if (!(across_norm > 1e-6 * std::max(1.0, math::norm(*up)))) {
        return invalid(table, "facing and up must not be parallel");
    }
    return PanelPlace{.position_m = *position, .facing = out, .up = across / across_norm};
}

[[nodiscard]] core::Result<std::vector<SignalSetting>> parse_settings(const TomlValue& table,
                                                                      std::string_view key) {
    const auto rows = table.array_or_empty(key);
    if (!rows) {
        return std::unexpected(rows.error());
    }
    std::vector<SignalSetting> settings;
    for (const TomlValue& row : *rows) {
        auto signal = row.string_at("signal");
        const auto value = row.number_at("value");
        if (const auto error = first_error(row.expect_keys({"signal", "value"}), signal, value)) {
            return std::unexpected(*error);
        }
        if (signal->empty() || !std::isfinite(*value)) {
            return invalid(row, "a setting needs a signal and a finite value");
        }
        settings.push_back(SignalSetting{.signal = std::move(*signal), .value = *value});
    }
    return settings;
}

// A switch's positions: numbers (values of its signal), or tables with a `value` of its
// signal, `commands` and `spring`.
[[nodiscard]] core::Result<std::vector<SwitchPosition>> parse_positions(const TomlValue& table,
                                                                        const std::string& signal) {
    const auto rows = table.array_or_empty("positions");
    if (!rows) {
        return std::unexpected(rows.error());
    }
    std::vector<SwitchPosition> positions;
    for (const TomlValue& row : *rows) {
        SwitchPosition position;
        std::optional<double> value;
        if (row.kind == TomlValue::Kind::Number) {
            value = row.number;
        } else if (row.kind == TomlValue::Kind::Table) {
            const auto spring = row.boolean_or("spring", false);
            auto settings = parse_settings(row, "commands");
            if (const auto error =
                    first_error(row.expect_keys({"value", "commands", "spring"}), spring, settings)) {
                return std::unexpected(*error);
            }
            if (row.contains("value")) {
                const auto number = row.number_at("value");
                if (!number) {
                    return std::unexpected(number.error());
                }
                value = *number;
            }
            position.spring = *spring;
            position.settings = std::move(*settings);
        } else {
            return invalid(table, "a switch's positions are numbers or tables");
        }
        if (value.has_value()) {
            if (signal.empty() || !std::isfinite(*value)) {
                return invalid(table, "a position's value needs the switch's signal, and to be finite");
            }
            position.settings.insert(position.settings.begin(),
                                     SignalSetting{.signal = signal, .value = *value});
        }
        if (position.settings.empty()) {
            return invalid(table, "each position of a switch sets something");
        }
        positions.push_back(std::move(position));
    }
    return positions;
}

[[nodiscard]] core::Result<std::optional<Guard>> parse_guard(const TomlValue& table) {
    if (!table.contains("guard")) {
        return std::optional<Guard>{};
    }
    const auto guard = table.at("guard");
    if (!guard) {
        return std::unexpected(guard.error());
    }
    const TomlValue& row = guard->get();
    if (row.kind == TomlValue::Kind::Boolean) {
        return row.boolean ? std::optional<Guard>{Guard{}} : std::optional<Guard>{};
    }
    if (row.kind != TomlValue::Kind::Table) {
        return invalid(table, "guard is true or a table");
    }
    if (const auto keys = row.expect_keys({"closes_to"}); !keys) {
        return std::unexpected(keys.error());
    }
    Guard made;
    if (row.contains("closes_to")) {
        const auto closes_to = row.number_at("closes_to");
        if (!closes_to) {
            return std::unexpected(closes_to.error());
        }
        if (!(*closes_to >= 0.0) || *closes_to != std::floor(*closes_to)) {
            return invalid(row, "closes_to is a switch's position: 0, 1, ...");
        }
        made.closes_to = static_cast<std::size_t>(*closes_to);
    }
    return std::optional<Guard>{made};
}

template <typename Type, std::size_t Count>
[[nodiscard]] std::optional<Type>
type_named(const std::array<std::pair<std::string_view, Type>, Count>& types, std::string_view name) {
    const auto match = std::ranges::find(types, name, &std::pair<std::string_view, Type>::first);
    return match != types.end() ? std::optional<Type>{match->second} : std::nullopt;
}

constexpr std::array<std::pair<std::string_view, Control::Type>, 5> k_control_types{{
    {"momentary", Control::Type::Momentary},
    {"switch", Control::Type::Switch},
    {"axis", Control::Type::Axis},
    {"stick", Control::Type::Stick},
    {"encoder", Control::Type::Encoder},
}};

constexpr std::array<std::pair<std::string_view, Indicator::Type>, 3> k_indicator_types{{
    {"gauge", Indicator::Type::Gauge},
    {"state", Indicator::Type::State},
    {"readout", Indicator::Type::Readout},
}};

[[nodiscard]] core::VoidResult check_control(const TomlValue& table, const Control& control) {
    using Type = Control::Type;
    const bool has_signal = !control.signal.empty();
    const auto fraction = [](double value) { return value >= 0.0 && value <= 1.0; };
    switch (control.type) {
    case Type::Momentary:
        if (const bool steps = has_signal && std::isfinite(control.step) && control.step != 0.0;
            control.press.empty() && !steps) {
            return invalid(table, "a momentary control needs commands, or a signal and a step");
        }
        break;
    case Type::Switch:
        if (control.positions.size() < 2) {
            return invalid(table, "a switch needs two positions or more");
        }
        if (control.rest >= control.positions.size() || control.positions[control.rest].spring) {
            return invalid(table, "a switch's rest is one of its positions, not a sprung one");
        }
        if (control.guard.has_value() && control.guard->closes_to.has_value()
            && *control.guard->closes_to >= control.positions.size()) {
            return invalid(table, "a guard closes_to one of the switch's positions");
        }
        break;
    case Type::Axis:
        if (!has_signal || !std::ranges::all_of(control.detents, fraction)
            || (control.spring_to.has_value() && !fraction(*control.spring_to))) {
            return invalid(table,
                           "an axis needs a signal; detents and spring_to are fractions of its travel");
        }
        break;
    case Type::Stick:
        if (control.signals.size() < 2 || control.signals.size() > 3 || control.guard.has_value()) {
            return invalid(table, "a stick moves two or three signals, and has no guard");
        }
        break;
    case Type::Encoder:
        if (!has_signal || !std::isfinite(control.step) || control.step == 0.0) {
            return invalid(table, "an encoder needs a signal and a step");
        }
        break;
    }
    const bool switch_only = control.locked || !control.positions.empty()
                             || (control.guard.has_value() && control.guard->closes_to.has_value());
    if (switch_only && control.type != Type::Switch) {
        return invalid(table, "positions, locked and a guard's closes_to are for a switch");
    }
    if ((!control.detents.empty() || control.spring_to.has_value()) && control.type != Type::Axis) {
        return invalid(table, "detents and spring_to are for an axis");
    }
    if ((!control.press.empty() || !control.release.empty()) && control.type != Type::Momentary) {
        return invalid(table, "commands and release are for a momentary control");
    }
    return {};
}

[[nodiscard]] const BuiltInShape* built_in(std::string_view element, std::string_view shape) {
    const auto match = std::ranges::find_if(k_shapes, [&](const BuiltInShape& entry) {
        return entry.element == element && entry.shape == shape;
    });
    return match != k_shapes.end() ? &*match : nullptr;
}

} // namespace

void LookPatch::apply(const LookPatch& other) {
    const auto over = [](auto& mine, const auto& theirs) {
        if (theirs.has_value()) {
            mine = theirs;
        }
    };
    over(shape, other.shape);
    over(size_m, other.size_m);
    over(travel, other.travel);
    over(colour, other.colour);
    over(accent, other.accent);
    over(paint, other.paint);
}

std::string_view element_name(Control::Type type) noexcept {
    return k_control_types.at(static_cast<std::size_t>(type)).first;
}

std::string_view element_name(Indicator::Type type) noexcept {
    return k_indicator_types.at(static_cast<std::size_t>(type)).first;
}

core::Result<StyleSheet> parse_style_sheet(std::string name, const TomlValue& document) {
    StyleSheet sheet{.name = std::move(name), .extends = {}, .rules = {}};
    for (std::size_t index = 0; index < document.keys.size(); ++index) {
        const std::string& key = document.keys[index];
        const TomlValue& table = document.items[index];
        if (table.kind != TomlValue::Kind::Table) {
            return invalid(table, std::format("'{}' in a style sheet is a table", key));
        }
        if (key == "style") {
            auto extends = table.string_or("extends", "");
            if (const auto error = first_error(table.expect_keys({"extends"}), extends)) {
                return std::unexpected(*error);
            }
            sheet.extends = std::move(*extends);
            continue;
        }
        if (std::ranges::find(k_elements, key) == k_elements.end()) {
            return invalid(table, std::format("unknown element '{}' (known: momentary, switch, axis, stick, "
                                              "encoder, guard, gauge, state, readout)",
                                              key));
        }
        // The element's own keys are its look; a table under it is a class.
        TomlValue own{.kind = TomlValue::Kind::Table, .line = table.line};
        for (std::size_t member = 0; member < table.keys.size(); ++member) {
            const TomlValue& value = table.items[member];
            if (value.kind == TomlValue::Kind::Table) {
                auto look = parse_look_patch(value);
                if (!look) {
                    return std::unexpected(look.error());
                }
                sheet.rules.push_back({.element = key, .class_name = table.keys[member], .look = *look});
            } else {
                own.keys.push_back(table.keys[member]);
                own.items.push_back(value);
            }
        }
        auto look = parse_look_patch(own);
        if (!look) {
            return std::unexpected(look.error());
        }
        sheet.rules.push_back({.element = key, .class_name = {}, .look = *look});
    }
    return sheet;
}

core::Result<Look> resolve_look(std::span<const StyleSheet> sheets, std::string_view style,
                                std::string_view element, std::span<const std::string> classes,
                                const LookPatch& own) {
    // The sheet and those it extends, most general first.
    std::vector<const StyleSheet*> chain;
    for (std::string_view name = style; !name.empty();) {
        const auto sheet = std::ranges::find(sheets, name, &StyleSheet::name);
        if (sheet == sheets.end()) {
            return core::fail(ErrorCode::InvalidArgument, std::format("no style sheet '{}'", name));
        }
        if (chain.size() > sheets.size()) {
            return core::fail(ErrorCode::InvalidArgument,
                              std::format("the style sheet '{}' extends itself, through others", style));
        }
        chain.insert(chain.begin(), &*sheet);
        name = sheet->extends;
    }
    // As in CSS, a rule for a class is above one for every element of the type, whichever sheet
    // each is in; the element's own look is above both.
    LookPatch patch;
    for (const StyleSheet* sheet : chain) {
        for (const StyleSheet::Rule& rule : sheet->rules) {
            if (rule.element == element && rule.class_name.empty()) {
                patch.apply(rule.look);
            }
        }
    }
    for (const std::string& class_name : classes) {
        for (const StyleSheet* sheet : chain) {
            for (const StyleSheet::Rule& rule : sheet->rules) {
                if (rule.element == element && rule.class_name == class_name) {
                    patch.apply(rule.look);
                }
            }
        }
    }
    patch.apply(own);
    const auto first = std::ranges::find(k_shapes, element, &BuiltInShape::element);
    if (first == k_shapes.end()) {
        return core::fail(ErrorCode::InvalidArgument, std::format("no element '{}'", element));
    }
    const std::string shape = patch.shape.value_or(std::string{first->shape});
    const BuiltInShape* base = built_in(element, shape);
    if (base == nullptr) {
        return core::fail(
            ErrorCode::InvalidArgument,
            std::format("a {} cannot have the shape '{}' (known: {})", element, shape, shapes_of(element)));
    }
    return Look{.shape = shape,
                .size_m = patch.size_m.value_or(base->size_m),
                .travel = patch.travel.value_or(base->travel),
                .colour = patch.colour.value_or(base->colour),
                .accent = patch.accent.value_or(base->accent),
                .paint = patch.paint.value_or(Look{}.paint)};
}

core::Result<Control> parse_control(const TomlValue& table) {
    auto id = table.string_at("id");
    const auto type_name = table.string_at("type");
    auto label = table.string_or("label", "");
    auto classes = classes_of(table);
    const auto place = parse_place(table);
    auto signal = table.string_or("signal", "");
    auto signals = table.strings_or_empty("signals");
    auto press = parse_settings(table, "commands");
    auto release = parse_settings(table, "release");
    const auto step = table.number_or("step", 0.0);
    const auto rest = table.number_or("rest", 0.0);
    const auto locked = table.boolean_or("locked", false);
    auto detents = table.numbers_or_empty("detents");
    const auto wrap = table.boolean_or("wrap", false);
    auto guard = parse_guard(table);
    auto look = look_or_empty(table);
    if (const auto error = first_error(
            table.expect_keys({"id",     "type",    "label",     "class",   "position_m", "facing",    "up",
                               "signal", "signals", "commands",  "release", "step",       "positions", "rest",
                               "locked", "detents", "spring_to", "wrap",    "guard",      "look"}),
            id, type_name, label, classes, place, signal, signals, press, release, step, rest, locked,
            detents, wrap, guard, look)) {
        return std::unexpected(*error);
    }
    const std::optional<Control::Type> type = type_named(k_control_types, *type_name);
    if (!type.has_value()) {
        return invalid(table, std::format("unknown control type '{}' (known: momentary, switch, axis, stick, "
                                          "encoder)",
                                          *type_name));
    }
    auto positions = parse_positions(table, *signal);
    if (!positions) {
        return std::unexpected(positions.error());
    }
    std::optional<double> spring_to;
    if (table.contains("spring_to")) {
        const auto value = table.number_at("spring_to");
        if (!value) {
            return std::unexpected(value.error());
        }
        spring_to = *value;
    }
    if (id->empty() || !(*rest >= 0.0) || *rest != std::floor(*rest)) {
        return invalid(table, "a control needs an id; its rest is a position: 0, 1, ...");
    }
    Control control{.id = std::move(*id),
                    .type = *type,
                    .label = std::move(*label),
                    .classes = std::move(*classes),
                    .place = *place,
                    .signal = std::move(*signal),
                    .signals = std::move(*signals),
                    .press = std::move(*press),
                    .release = std::move(*release),
                    .step = *step,
                    .positions = std::move(*positions),
                    .rest = static_cast<std::size_t>(*rest),
                    .locked = *locked,
                    .detents = std::move(*detents),
                    .spring_to = spring_to,
                    .wrap = *wrap,
                    .guard = *guard,
                    .own_look = *look,
                    .look = {}};
    if (core::VoidResult checked = check_control(table, control); !checked) {
        return std::unexpected(checked.error());
    }
    return control;
}

core::Result<Indicator> parse_indicator(const TomlValue& table) {
    const Indicator defaults;
    auto id = table.string_at("id");
    const auto type_name = table.string_at("type");
    auto label = table.string_or("label", "");
    auto classes = classes_of(table);
    const auto place = parse_place(table);
    auto signal = table.string_at("signal");
    const auto minimum = table.number_or("minimum", defaults.minimum);
    const auto maximum = table.number_or("maximum", defaults.maximum);
    const auto scale = table.number_or("scale", defaults.scale);
    auto unit = table.string_or("unit", "");
    auto thresholds = table.numbers_or_empty("thresholds");
    auto look = look_or_empty(table);
    if (const auto error = first_error(
            table.expect_keys({"id", "type", "label", "class", "position_m", "facing", "up", "signal",
                               "minimum", "maximum", "scale", "unit", "thresholds", "look"}),
            id, type_name, label, classes, place, signal, minimum, maximum, scale, unit, thresholds, look)) {
        return std::unexpected(*error);
    }
    const std::optional<Indicator::Type> type = type_named(k_indicator_types, *type_name);
    if (!type.has_value()) {
        return invalid(table,
                       std::format("unknown indicator type '{}' (known: gauge, state, readout)", *type_name));
    }
    if (id->empty() || signal->empty() || !std::isfinite(*scale)) {
        return invalid(table, "an indicator needs an id, a signal and a finite scale");
    }
    if (*type == Indicator::Type::Gauge
        && (!std::isfinite(*minimum) || !std::isfinite(*maximum) || !(*minimum < *maximum))) {
        return invalid(table, "a gauge needs minimum < maximum");
    }
    if (*type == Indicator::Type::State) {
        if (thresholds->empty()) {
            thresholds->push_back(0.5); // a lamp, lit from one half on
        }
        if (!std::ranges::is_sorted(*thresholds)
            || !std::ranges::all_of(*thresholds, [](double value) { return std::isfinite(value); })) {
            return invalid(table, "thresholds are finite and in increasing order");
        }
    } else if (!thresholds->empty()) {
        return invalid(table, "thresholds are for a state indicator");
    }
    return Indicator{.id = std::move(*id),
                     .type = *type,
                     .label = std::move(*label),
                     .classes = std::move(*classes),
                     .place = *place,
                     .signal = std::move(*signal),
                     .minimum = *minimum,
                     .maximum = *maximum,
                     .scale = *scale,
                     .unit = std::move(*unit),
                     .thresholds = std::move(*thresholds),
                     .own_look = *look,
                     .look = {}};
}

} // namespace helios::vessel
