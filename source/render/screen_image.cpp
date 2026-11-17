#include "helios/render/screen_image.hpp"

#include "helios/render/stroke_font.hpp"
#include "helios/vessel/cockpit.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <functional>
#include <iterator>
#include <string>
#include <string_view>

namespace helios::render {

namespace {

using vessel::ScreenElement;

// Rationale: glass that is lit shows a little of its own dark; glass that is not, the cabin's grey.
constexpr std::uint32_t k_lit_glass = rgba(6, 10, 8);
constexpr std::uint32_t k_dark_glass = rgba(18, 19, 21);

[[nodiscard]] std::uint32_t colour_of(const math::Vector3& colour) noexcept {
    const auto channel = [](double value) {
        return static_cast<std::uint8_t>(std::lround(std::clamp(value, 0.0, 1.0) * 255.0));
    };
    return rgba(channel(colour.x), channel(colour.y), channel(colour.z));
}

[[nodiscard]] const sim::SignalView* find_signal(const sim::VesselView& vessel, std::string_view name) {
    const auto match = std::ranges::find(vessel.signals, name, &sim::SignalView::name);
    return match != vessel.signals.end() ? &*match : nullptr;
}

// A number with `decimals`, or with as many as its size leaves room for.
[[nodiscard]] std::string format_number(double value, std::optional<int> decimals, std::string_view unit) {
    const double magnitude = std::abs(value);
    int shown_decimals = 0;
    if (decimals.has_value()) {
        shown_decimals = *decimals;
    } else if (magnitude < 10.0) {
        shown_decimals = 2;
    } else if (magnitude < 100.0) {
        shown_decimals = 1;
    }
    // Rationale: a value that rounds to zero would otherwise read "-0.00".
    const double shown = magnitude < 0.5 * std::pow(10.0, -shown_decimals) ? 0.0 : value;
    return unit.empty() ? std::format("{:.{}f}", shown, shown_decimals)
                        : std::format("{:.{}f} {}", shown, shown_decimals, unit);
}

// Draws in a screen's pixels: x to the right, y down, from the top left.
class Painter {
public:
    explicit Painter(std::vector<LineVertex>& triangles) noexcept : triangles_(triangles) {}

    void box(double x0, double y0, double x1, double y1, std::uint32_t abgr) {
        const LineVertex a = vertex(x0, y0, abgr);
        const LineVertex b = vertex(x1, y0, abgr);
        const LineVertex c = vertex(x1, y1, abgr);
        const LineVertex d = vertex(x0, y1, abgr);
        triangles_.get().insert(triangles_.get().end(), {a, b, c, a, c, d});
    }

    void frame(double x0, double y0, double x1, double y1, double width, std::uint32_t abgr) {
        box(x0, y0, x1, y0 + width, abgr);
        box(x0, y1 - width, x1, y1, abgr);
        box(x0, y0 + width, x0 + width, y1 - width, abgr);
        box(x1 - width, y0 + width, x1, y1 - width, abgr);
    }

    // A stroke `width` wide, its ends going on by half of it so that strokes meeting close the
    // corner.
    void line(double x0, double y0, double x1, double y1, double width, std::uint32_t abgr) {
        const double length = std::hypot(x1 - x0, y1 - y0);
        const double along_x = length > 0.0 ? (x1 - x0) / length : 1.0;
        const double along_y = length > 0.0 ? (y1 - y0) / length : 0.0;
        const double half = 0.5 * width;
        const double ax = x0 - (half * along_x);
        const double ay = y0 - (half * along_y);
        const double bx = x1 + (half * along_x);
        const double by = y1 + (half * along_y);
        const double across_x = -half * along_y;
        const double across_y = half * along_x;
        const LineVertex a = vertex(ax + across_x, ay + across_y, abgr);
        const LineVertex b = vertex(ax - across_x, ay - across_y, abgr);
        const LineVertex c = vertex(bx - across_x, by - across_y, abgr);
        const LineVertex d = vertex(bx + across_x, by + across_y, abgr);
        triangles_.get().insert(triangles_.get().end(), {a, b, c, a, c, d});
    }

    // `x` is the text's left, centre or right as `align` says; `top` the top of its capitals.
    void text(std::string_view words, double x, double top, double height, ScreenElement::Align align,
              std::uint32_t abgr) {
        const double width = static_cast<double>(text_width(words)) * height;
        double centre_x = x + (0.5 * width);
        if (align == ScreenElement::Align::Centre) {
            centre_x = x;
        } else if (align == ScreenElement::Align::Right) {
            centre_x = x - (0.5 * width);
        }
        append_text(triangles_, words,
                    TextPlacement{.centre_m = {centre_x, top + (0.5 * height), 0.0},
                                  .right_m = {height, 0.0, 0.0},
                                  .up_m = {0.0, -height, 0.0}},
                    abgr);
    }

private:
    [[nodiscard]] static LineVertex vertex(double x, double y, std::uint32_t abgr) noexcept {
        return LineVertex{.position = {static_cast<float>(x), static_cast<float>(y), 0.0F}, .abgr = abgr};
    }

    std::reference_wrapper<std::vector<LineVertex>> triangles_;
};

[[nodiscard]] double fraction_between(double value, double minimum, double maximum) noexcept {
    return std::clamp((value - minimum) / (maximum - minimum), 0.0, 1.0);
}

void draw_graph(Painter& painter, const ScreenElement& element, std::span<const GraphSample> samples,
                double now_s, std::uint32_t ink) {
    const double x0 = element.at_px[0];
    const double y0 = element.at_px[1];
    const double width = element.size_px[0];
    const double height = element.size_px[1];
    painter.frame(x0, y0, x0 + width, y0 + height, 1.0, ink);
    // Now is at the right edge; the left edge is `span_s` ago.
    const auto point = [&](const GraphSample& sample) {
        const double across = std::clamp(1.0 - ((now_s - sample.time_s) / element.span_s), 0.0, 1.0);
        const double up = fraction_between(sample.value, element.minimum, element.maximum);
        return std::array{x0 + (across * width), y0 + ((1.0 - up) * height)};
    };
    for (std::size_t index = 1; index < samples.size(); ++index) {
        if (now_s - samples[index].time_s > element.span_s) {
            continue;
        }
        const auto from = point(samples[index - 1]);
        const auto to = point(samples[index]);
        painter.line(from[0], from[1], to[0], to[1], element.line_px, ink);
    }
}

} // namespace

void ScreenMemory::record(const sim::SceneSnapshot& snapshot) {
    if (!origin_.has_value()) {
        origin_ = snapshot.epoch;
    }
    double time_s = time::seconds_between(origin_.value_or(snapshot.epoch), snapshot.epoch);
    if (time_s < now_s_) {
        traces_.clear();
        origin_ = snapshot.epoch;
        time_s = 0.0;
    }
    now_s_ = time_s;
    for (const sim::VesselView& vessel : snapshot.vessels) {
        for (std::size_t part = 0; part < vessel.parts.size(); ++part) {
            const auto& datasheet = vessel.parts[part].datasheet;
            if (datasheet == nullptr) {
                continue;
            }
            const std::optional<vessel::Cockpit>& cockpit = datasheet->cockpit;
            if (!cockpit.has_value()) {
                continue;
            }
            const std::vector<vessel::Screen>& screens = cockpit->screens;
            for (std::size_t screen = 0; screen < screens.size(); ++screen) {
                for (std::size_t page = 0; page < screens[screen].pages.size(); ++page) {
                    const std::vector<ScreenElement>& elements = screens[screen].pages[page].elements;
                    for (std::size_t element = 0; element < elements.size(); ++element) {
                        const ScreenElement& graph = elements[element];
                        const sim::SignalView* signal = find_signal(vessel, graph.signal);
                        if (graph.type != ScreenElement::Type::Graph || signal == nullptr) {
                            continue;
                        }
                        const GraphKey key{.vessel = vessel.id,
                                           .part = part,
                                           .screen = screen,
                                           .page = page,
                                           .element = element};
                        auto trace = std::ranges::find(traces_, key, &Trace::key);
                        if (trace == traces_.end()) {
                            traces_.push_back(Trace{.key = key, .samples = {}});
                            trace = std::prev(traces_.end());
                        }
                        std::vector<GraphSample>& samples = trace->samples;
                        const double spacing_s = graph.span_s / static_cast<double>(k_samples);
                        if (samples.empty() || time_s - samples.back().time_s >= spacing_s) {
                            samples.push_back({.time_s = time_s, .value = signal->value});
                        }
                        // One sample older than the span stays, for the trace to reach the edge.
                        const auto kept = std::ranges::find_if(samples, [&](const GraphSample& sample) {
                            return time_s - sample.time_s <= graph.span_s;
                        });
                        if (kept - samples.begin() > 1) {
                            samples.erase(samples.begin(), std::prev(kept));
                        }
                    }
                }
            }
        }
    }
}

std::span<const GraphSample> ScreenMemory::samples(const GraphKey& key) const noexcept {
    const auto trace = std::ranges::find(traces_, key, &Trace::key);
    return trace != traces_.end() ? std::span<const GraphSample>{trace->samples}
                                  : std::span<const GraphSample>{};
}

ScreenImage draw_screen(const vessel::Screen& screen, const sim::VesselView& vessel,
                        const ScreenMemory* memory, std::size_t part, std::size_t screen_index) {
    ScreenImage image{.width_px = screen.resolution_px[0],
                      .height_px = screen.resolution_px[1],
                      .on = false,
                      .background = k_dark_glass,
                      .triangles = {}};
    const sim::SignalView* power =
        screen.power_signal.empty() ? nullptr : find_signal(vessel, screen.power_signal);
    const sim::SignalView* page_signal =
        screen.page_signal.empty() ? nullptr : find_signal(vessel, screen.page_signal);
    const bool powered = screen.power_signal.empty() || (power != nullptr && power->value >= 0.5);
    const bool paged = screen.page_signal.empty() || page_signal != nullptr;
    if (!powered || !paged || screen.pages.empty()) {
        return image;
    }
    image.on = true;
    image.background = k_lit_glass;
    const std::size_t page = page_signal != nullptr ? vessel::page_shown(screen, page_signal->value) : 0;
    Painter painter(image.triangles);
    const std::vector<ScreenElement>& elements = screen.pages[page].elements;
    for (std::size_t index = 0; index < elements.size(); ++index) {
        const ScreenElement& element = elements[index];
        const std::uint32_t ink = colour_of(element.colour.value_or(screen.look.accent));
        const sim::SignalView* signal =
            element.signal.empty() ? nullptr : find_signal(vessel, element.signal);
        const double x0 = element.at_px[0];
        const double y0 = element.at_px[1];
        const double x1 = x0 + element.size_px[0];
        const double y1 = y0 + element.size_px[1];
        switch (element.type) {
        case ScreenElement::Type::Text:
            painter.text(element.text, x0, y0, element.height_px, element.align, ink);
            break;
        case ScreenElement::Type::Readout: {
            const std::string value = signal != nullptr ? format_number(signal->value * element.scale,
                                                                        element.decimals, element.unit)
                                                        : std::string{"----"};
            const std::string words =
                element.text.empty() ? value : std::format("{} {}", element.text, value);
            painter.text(words, x0, y0, element.height_px, element.align, ink);
            break;
        }
        case ScreenElement::Type::Bar: {
            painter.frame(x0, y0, x1, y1, element.line_px, ink);
            if (signal == nullptr) {
                break;
            }
            const double fraction = fraction_between(signal->value, element.minimum, element.maximum);
            // Taller than wide fills from the bottom; otherwise from the left.
            if (element.size_px[1] > element.size_px[0]) {
                painter.box(x0, y1 - (fraction * element.size_px[1]), x1, y1, ink);
            } else {
                painter.box(x0, y0, x0 + (fraction * element.size_px[0]), y1, ink);
            }
            break;
        }
        case ScreenElement::Type::Graph:
            draw_graph(painter, element,
                       memory != nullptr ? memory->samples(GraphKey{.vessel = vessel.id,
                                                                    .part = part,
                                                                    .screen = screen_index,
                                                                    .page = page,
                                                                    .element = index})
                                         : std::span<const GraphSample>{},
                       memory != nullptr ? memory->now_s() : 0.0, ink);
            break;
        case ScreenElement::Type::Box:
            if (element.fill) {
                painter.box(x0, y0, x1, y1, ink);
            } else {
                painter.frame(x0, y0, x1, y1, element.line_px, ink);
            }
            break;
        case ScreenElement::Type::Line:
            painter.line(x0, y0, element.to_px[0], element.to_px[1], element.line_px, ink);
            break;
        }
    }
    return image;
}

} // namespace helios::render
