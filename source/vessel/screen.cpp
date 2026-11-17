#include "helios/vessel/screen.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <string_view>
#include <utility>

#include "panel_parse.hpp"
#include "toml_vector.hpp"

namespace helios::vessel {

namespace {

using core::ErrorCode;
using core::first_error;
using core::TomlValue;
using math::Vector3;

[[nodiscard]] std::unexpected<core::Error> invalid(const TomlValue& table, std::string_view message) {
    return core::fail(ErrorCode::ParseFailure, std::format("line {}: {}", table.line, message));
}

[[nodiscard]] bool is_colour(const Vector3& colour) noexcept {
    return std::ranges::all_of(std::array{colour.x, colour.y, colour.z},
                               [](double channel) { return channel >= 0.0 && channel <= 1.0; });
}

// A member of two numbers; `fallback` when the key is absent.
[[nodiscard]] core::Result<std::array<double, 2>> pair_or(const TomlValue& table, std::string_view key,
                                                          std::array<double, 2> fallback) {
    if (!table.contains(key)) {
        return fallback;
    }
    return table.numbers_or_empty(key).and_then(
        [&](const std::vector<double>& numbers) -> core::Result<std::array<double, 2>> {
            if (numbers.size() != 2 || !std::isfinite(numbers[0]) || !std::isfinite(numbers[1])) {
                return invalid(table, std::format("'{}' must have 2 finite numbers", key));
            }
            return std::array{numbers[0], numbers[1]};
        });
}

constexpr std::array<std::pair<std::string_view, ScreenElement::Type>, 8> k_element_types{{
    {"text", ScreenElement::Type::Text},
    {"readout", ScreenElement::Type::Readout},
    {"bar", ScreenElement::Type::Bar},
    {"graph", ScreenElement::Type::Graph},
    {"box", ScreenElement::Type::Box},
    {"line", ScreenElement::Type::Line},
    {"attitude", ScreenElement::Type::Attitude},
    {"orbit", ScreenElement::Type::Orbit},
}};

constexpr std::array<std::pair<std::string_view, ScreenElement::Align>, 3> k_alignments{{
    {"left", ScreenElement::Align::Left},
    {"centre", ScreenElement::Align::Centre},
    {"right", ScreenElement::Align::Right},
}};

template <typename Type, std::size_t Count>
[[nodiscard]] std::optional<Type> named(const std::array<std::pair<std::string_view, Type>, Count>& names,
                                        std::string_view name) {
    const auto match = std::ranges::find(names, name, &std::pair<std::string_view, Type>::first);
    return match != names.end() ? std::optional<Type>{match->second} : std::nullopt;
}

// Rationale: the most a readout writes, so that a mistake does not fill the screen with digits.
constexpr int k_most_decimals = 9;

[[nodiscard]] core::VoidResult check_element(const TomlValue& table, const ScreenElement& element) {
    using Type = ScreenElement::Type;
    const bool positive_size = element.size_px[0] > 0.0 && element.size_px[1] > 0.0;
    const bool range =
        std::isfinite(element.minimum) && std::isfinite(element.maximum) && element.minimum < element.maximum;
    switch (element.type) {
    case Type::Text:
        if (element.text.empty()) {
            return invalid(table, "a text element needs a text");
        }
        break;
    case Type::Readout:
        if (element.signal.empty() || !std::isfinite(element.scale)) {
            return invalid(table, "a readout needs a signal and a finite scale");
        }
        break;
    case Type::Bar:
    case Type::Graph:
        if (element.signal.empty() || !positive_size || !range) {
            return invalid(table,
                           "a bar or a graph needs a signal, a positive size_px and minimum < maximum");
        }
        if (element.type == Type::Graph && (element.span_s <= 0.0 || !std::isfinite(element.span_s))) {
            return invalid(table, "a graph's span_s is a positive number of seconds");
        }
        break;
    case Type::Box:
    case Type::Attitude:
    case Type::Orbit:
        if (!positive_size) {
            return invalid(table, "a box, an attitude ball or an orbit needs a positive size_px");
        }
        break;
    case Type::Line: break;
    }
    if (!(element.height_px > 0.0) || !std::isfinite(element.height_px) || !(element.line_px > 0.0)
        || !std::isfinite(element.line_px)) {
        return invalid(table, "height_px and line_px are positive");
    }
    if (element.decimals.has_value() && (*element.decimals < 0 || *element.decimals > k_most_decimals)) {
        return invalid(table, std::format("decimals is a whole number from 0 to {}", k_most_decimals));
    }
    if (element.colour.has_value() && !is_colour(*element.colour)) {
        return invalid(table, "a colour is three numbers from 0 to 1");
    }
    return {};
}

[[nodiscard]] core::Result<ScreenElement> parse_element(const TomlValue& table) {
    const ScreenElement defaults;
    const auto type_name = table.string_at("type");
    const auto at = pair_or(table, "at_px", defaults.at_px);
    const auto size = pair_or(table, "size_px", defaults.size_px);
    const auto to = pair_or(table, "to_px", defaults.to_px);
    auto text = table.string_or("text", "");
    auto signal = table.string_or("signal", "");
    const auto minimum = table.number_or("minimum", defaults.minimum);
    const auto maximum = table.number_or("maximum", defaults.maximum);
    const auto scale = table.number_or("scale", defaults.scale);
    auto unit = table.string_or("unit", "");
    const auto decimals = table.number_or("decimals", -1.0);
    const auto height = table.number_or("height_px", defaults.height_px);
    const auto align_name = table.string_or("align", "left");
    const auto span = table.number_or("span_s", defaults.span_s);
    const auto line = table.number_or("line_px", defaults.line_px);
    const auto fill = table.boolean_or("fill", defaults.fill);
    const auto colour = detail::vector_or(table, "colour", Vector3{-1.0, -1.0, -1.0});
    if (const auto error =
            first_error(table.expect_keys({"type", "at_px", "size_px", "to_px", "text", "signal", "minimum",
                                           "maximum", "scale", "unit", "decimals", "height_px", "align",
                                           "span_s", "line_px", "fill", "colour"}),
                        type_name, at, size, to, text, signal, minimum, maximum, scale, unit, decimals,
                        height, align_name, span, line, fill, colour)) {
        return std::unexpected(*error);
    }
    const std::optional<ScreenElement::Type> type = named(k_element_types, *type_name);
    if (!type.has_value()) {
        return invalid(table, std::format("unknown screen element '{}' (known: text, readout, bar, graph, "
                                          "box, line)",
                                          *type_name));
    }
    const std::optional<ScreenElement::Align> align = named(k_alignments, *align_name);
    if (!align.has_value()) {
        return invalid(table, "align is left, centre or right");
    }
    if (table.contains("decimals") && (*decimals != std::floor(*decimals) || !std::isfinite(*decimals))) {
        return invalid(table, "decimals is a whole number");
    }
    ScreenElement element{
        .type = *type,
        .at_px = *at,
        .size_px = *size,
        .to_px = *to,
        .text = std::move(*text),
        .signal = std::move(*signal),
        .minimum = *minimum,
        .maximum = *maximum,
        .scale = *scale,
        .unit = std::move(*unit),
        .decimals =
            table.contains("decimals") ? std::optional<int>{static_cast<int>(*decimals)} : std::nullopt,
        .height_px = *height,
        .align = *align,
        .span_s = *span,
        .line_px = *line,
        .fill = *fill,
        .colour = table.contains("colour") ? std::optional<Vector3>{*colour} : std::nullopt,
    };
    if (core::VoidResult checked = check_element(table, element); !checked) {
        return std::unexpected(checked.error());
    }
    return element;
}

[[nodiscard]] core::Result<ScreenPage> parse_page(const TomlValue& table) {
    auto name = table.string_or("name", "");
    const auto rows = table.array_or_empty("elements");
    if (const auto error = first_error(table.expect_keys({"name", "elements"}), name, rows)) {
        return std::unexpected(*error);
    }
    ScreenPage page{.name = std::move(*name), .elements = {}};
    for (const TomlValue& row : *rows) {
        auto element = parse_element(row);
        if (!element) {
            return std::unexpected(element.error());
        }
        page.elements.push_back(std::move(*element));
    }
    return page;
}

[[nodiscard]] bool is_resolution(double pixels) noexcept {
    return pixels == std::floor(pixels) && pixels >= k_least_screen_px && pixels <= k_most_screen_px;
}

} // namespace

core::Result<Screen> parse_screen(const TomlValue& table) {
    auto id = table.string_at("id");
    auto label = table.string_or("label", "");
    auto classes = detail::classes_of(table);
    const auto place = detail::parse_place(table);
    const auto size = pair_or(table, "size_m", {});
    const auto resolution = pair_or(table, "resolution_px", {});
    auto page_signal = table.string_or("page_signal", "");
    auto power_signal = table.string_or("power_signal", "");
    const auto pages = table.array_or_empty("page");
    auto look = detail::look_or_empty(table);
    if (const auto error = first_error(
            table.expect_keys({"id", "label", "class", "position_m", "facing", "up", "size_m",
                               "resolution_px", "page_signal", "power_signal", "page", "look"}),
            id, label, classes, place, size, resolution, page_signal, power_signal, pages, look)) {
        return std::unexpected(*error);
    }
    if (id->empty()) {
        return invalid(table, "a screen needs an id");
    }
    if (!((*size)[0] > 0.0) || !((*size)[1] > 0.0)) {
        return invalid(table, "a screen needs a positive size_m, across and up");
    }
    if (!is_resolution((*resolution)[0]) || !is_resolution((*resolution)[1])) {
        return invalid(table, std::format("a screen's resolution_px is two whole numbers from {} to {}",
                                          k_least_screen_px, k_most_screen_px));
    }
    if (pages->empty()) {
        return invalid(table, "a screen needs a page: a [[...screen.page]] table");
    }
    Screen screen{.id = std::move(*id),
                  .label = std::move(*label),
                  .classes = std::move(*classes),
                  .place = *place,
                  .size_m = *size,
                  .resolution_px = {static_cast<std::uint16_t>((*resolution)[0]),
                                    static_cast<std::uint16_t>((*resolution)[1])},
                  .page_signal = std::move(*page_signal),
                  .power_signal = std::move(*power_signal),
                  .pages = {},
                  .own_look = *look,
                  .look = {}};
    for (const TomlValue& row : *pages) {
        auto shown = parse_page(row);
        if (!shown) {
            return std::unexpected(shown.error());
        }
        screen.pages.push_back(std::move(*shown));
    }
    return screen;
}

std::size_t page_shown(const Screen& screen, double page_value) noexcept {
    if (screen.pages.empty() || !std::isfinite(page_value)) {
        return 0;
    }
    const auto last = static_cast<double>(screen.pages.size() - 1);
    return static_cast<std::size_t>(std::clamp(std::round(page_value), 0.0, last));
}

} // namespace helios::vessel
