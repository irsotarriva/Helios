#include "helios/render/screen_image.hpp"

#include "helios/render/stroke_font.hpp"
#include "helios/vessel/cockpit.hpp"
#include "helios/vessel/vessel_systems.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <iterator>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

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

    void triangle(const std::array<double, 2>& a, const std::array<double, 2>& b,
                  const std::array<double, 2>& c, std::uint32_t abgr) {
        triangles_.get().insert(triangles_.get().end(), {vertex(a[0], a[1], abgr), vertex(b[0], b[1], abgr),
                                                         vertex(c[0], c[1], abgr)});
    }

    // A ring of `segments` strokes about (x, y).
    void ring(double x, double y, double radius, double width, std::uint32_t abgr) {
        constexpr int k_segments = 64;
        for (int index = 0; index < k_segments; ++index) {
            const double from = 2.0 * std::numbers::pi * index / k_segments;
            const double to = 2.0 * std::numbers::pi * (index + 1) / k_segments;
            line(x + (radius * std::cos(from)), y + (radius * std::sin(from)), x + (radius * std::cos(to)),
                 y + (radius * std::sin(to)), width, abgr);
        }
    }

    void disc(double x, double y, double radius, std::uint32_t abgr) {
        constexpr int k_segments = 48;
        for (int index = 0; index < k_segments; ++index) {
            const double from = 2.0 * std::numbers::pi * index / k_segments;
            const double to = 2.0 * std::numbers::pi * (index + 1) / k_segments;
            triangle({x, y}, {x + (radius * std::cos(from)), y + (radius * std::sin(from))},
                     {x + (radius * std::cos(to)), y + (radius * std::sin(to))}, abgr);
        }
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

// A value a screen can show: there, and a number.
[[nodiscard]] std::optional<double> shown_value(const sim::SignalView* signal) noexcept {
    if (signal == nullptr || !std::isfinite(signal->value)) {
        return std::nullopt;
    }
    return signal->value;
}

[[nodiscard]] std::uint32_t dimmed(const math::Vector3& colour, double brightness) noexcept {
    return colour_of(brightness * colour);
}

// The attitude ball (D47), drawn as the pilot sees it: the ball keeps to the local horizon and
// the pilot looks at it along the nose, with the vessel's top up. Directions are in east, north,
// up.
class AttitudeBall {
public:
    AttitudeBall(double pitch_rad, double heading_rad, double roll_rad, double centre_x, double centre_y,
                 double radius) noexcept
        : centre_x_(centre_x), centre_y_(centre_y), radius_(radius) {
        const double cos_pitch = std::cos(pitch_rad);
        const double sin_pitch = std::sin(pitch_rad);
        const double cos_heading = std::cos(heading_rad);
        const double sin_heading = std::sin(heading_rad);
        nose_ = {cos_pitch * sin_heading, cos_pitch * cos_heading, sin_pitch};
        const math::Vector3 level{-sin_pitch * sin_heading, -sin_pitch * cos_heading, cos_pitch};
        const math::Vector3 level_right = math::cross(nose_, level);
        top_ = (std::cos(roll_rad) * level) + (std::sin(roll_rad) * level_right);
        right_ = (std::cos(roll_rad) * level_right) - (std::sin(roll_rad) * level);
    }

    // A point of the ball at `heading_rad` from north and `pitch_rad` above the horizon.
    [[nodiscard]] static math::Vector3 at(double heading_rad, double pitch_rad) noexcept {
        return {std::cos(pitch_rad) * std::sin(heading_rad), std::cos(pitch_rad) * std::cos(heading_rad),
                std::sin(pitch_rad)};
    }
    // Towards the pilot: positive on the near side.
    [[nodiscard]] double depth(const math::Vector3& point) const noexcept { return math::dot(point, nose_); }
    [[nodiscard]] std::array<double, 2> pixel(const math::Vector3& point) const noexcept {
        return {centre_x_ + (radius_ * math::dot(point, right_)),
                centre_y_ - (radius_ * math::dot(point, top_))};
    }

    // The part of a flat polygon of the ball's points on the near side, filled.
    void fill(Painter& painter, std::span<const math::Vector3> corners, std::uint32_t abgr) const {
        std::vector<math::Vector3> kept;
        for (std::size_t index = 0; index < corners.size(); ++index) {
            const math::Vector3& from = corners[index];
            const math::Vector3& to = corners[(index + 1) % corners.size()];
            const double from_depth = depth(from);
            const double to_depth = depth(to);
            if (from_depth >= 0.0) {
                kept.push_back(from);
            }
            if ((from_depth >= 0.0) != (to_depth >= 0.0)) {
                kept.push_back(from + ((from_depth / (from_depth - to_depth)) * (to - from)));
            }
        }
        for (std::size_t index = 2; index < kept.size(); ++index) {
            painter.triangle(pixel(kept[0]), pixel(kept[index - 1]), pixel(kept[index]), abgr);
        }
    }

    // A stroke between two of the ball's points, where it is on the near side.
    void stroke(Painter& painter, math::Vector3 from, math::Vector3 to, double width,
                std::uint32_t abgr) const {
        const double from_depth = depth(from);
        const double to_depth = depth(to);
        if (from_depth < 0.0 && to_depth < 0.0) {
            return;
        }
        if (from_depth < 0.0) {
            from = from + ((from_depth / (from_depth - to_depth)) * (to - from));
        } else if (to_depth < 0.0) {
            to = to + ((to_depth / (to_depth - from_depth)) * (from - to));
        }
        const auto a = pixel(from);
        const auto b = pixel(to);
        painter.line(a[0], a[1], b[0], b[1], width, abgr);
    }

private:
    double centre_x_;
    double centre_y_;
    double radius_;
    math::Vector3 nose_;
    math::Vector3 top_;
    math::Vector3 right_;
};

constexpr double k_degree_rad = std::numbers::pi / 180.0;

// The ground, darker than the sky, in cells of 15 degrees; circles of pitch every 30 degrees
// (the horizon wider and brighter) and meridians every 30 degrees.
void draw_ball_surface(Painter& painter, const AttitudeBall& ball, const ScreenElement& element,
                       const math::Vector3& colour) {
    const std::uint32_t ink = colour_of(colour);
    const std::uint32_t ground = dimmed(colour, 0.3);
    for (int heading_deg = 0; heading_deg < 360; heading_deg += 15) {
        for (int pitch_deg = -90; pitch_deg < 0; pitch_deg += 15) {
            const double h0 = heading_deg * k_degree_rad;
            const double h1 = (heading_deg + 15) * k_degree_rad;
            const double p0 = pitch_deg * k_degree_rad;
            const double p1 = (pitch_deg + 15) * k_degree_rad;
            const std::array<math::Vector3, 4> cell{AttitudeBall::at(h0, p0), AttitudeBall::at(h1, p0),
                                                    AttitudeBall::at(h1, p1), AttitudeBall::at(h0, p1)};
            ball.fill(painter, cell, ground);
        }
    }
    const std::uint32_t grid = dimmed(colour, 0.6);
    for (int pitch_deg = -60; pitch_deg <= 60; pitch_deg += 30) {
        const bool horizon = pitch_deg == 0;
        for (int heading_deg = 0; heading_deg < 360; heading_deg += 5) {
            ball.stroke(painter, AttitudeBall::at(heading_deg * k_degree_rad, pitch_deg * k_degree_rad),
                        AttitudeBall::at((heading_deg + 5) * k_degree_rad, pitch_deg * k_degree_rad),
                        horizon ? 2.0 * element.line_px : element.line_px, horizon ? ink : grid);
        }
    }
    for (int heading_deg = 0; heading_deg < 360; heading_deg += 30) {
        for (int pitch_deg = -80; pitch_deg < 80; pitch_deg += 5) {
            ball.stroke(painter, AttitudeBall::at(heading_deg * k_degree_rad, pitch_deg * k_degree_rad),
                        AttitudeBall::at(heading_deg * k_degree_rad, (pitch_deg + 5) * k_degree_rad),
                        element.line_px, grid);
        }
    }
}

void draw_attitude(Painter& painter, const ScreenElement& element, const sim::VesselView& vessel,
                   const math::Vector3& colour) {
    const std::uint32_t ink = colour_of(colour);
    const double radius = 0.5 * std::min(element.size_px[0], element.size_px[1]);
    const double x = element.at_px[0] + (0.5 * element.size_px[0]);
    const double y = element.at_px[1] + (0.5 * element.size_px[1]);
    const auto pitch = shown_value(find_signal(vessel, vessel::k_signal_pitch));
    const auto heading = shown_value(find_signal(vessel, vessel::k_signal_heading));
    const auto roll = shown_value(find_signal(vessel, vessel::k_signal_roll));
    if (!pitch || !heading || !roll) {
        painter.ring(x, y, radius, element.line_px, ink);
        painter.text("----", x, y - (0.5 * element.height_px), element.height_px,
                     ScreenElement::Align::Centre, ink);
        return;
    }
    const AttitudeBall ball(*pitch, *heading, *roll, x, y, radius);
    draw_ball_surface(painter, ball, element, colour);
    // The cardinal points, a little above the horizon.
    constexpr std::array<std::string_view, 4> k_cardinals{"N", "E", "S", "W"};
    constexpr double k_least_depth = 0.3; // nearer the rim the letters would be squeezed
    for (std::size_t index = 0; index < k_cardinals.size(); ++index) {
        const math::Vector3 point =
            AttitudeBall::at(static_cast<double>(index) * 0.5 * std::numbers::pi, 4.0 * k_degree_rad);
        if (ball.depth(point) > k_least_depth) {
            const auto at = ball.pixel(point);
            painter.text(k_cardinals.at(index), at[0], at[1] - element.height_px, element.height_px,
                         ScreenElement::Align::Centre, ink);
        }
    }
    // The vessel, fixed: its wings and its nose; and the ball's rim.
    constexpr double k_wing_root = 0.18;
    constexpr double k_wing_tip = 0.55;
    constexpr double k_nose_px = 1.5;
    painter.line(x - (k_wing_tip * radius), y, x - (k_wing_root * radius), y, 2.0 * element.line_px, ink);
    painter.line(x + (k_wing_root * radius), y, x + (k_wing_tip * radius), y, 2.0 * element.line_px, ink);
    painter.box(x - k_nose_px, y - k_nose_px, x + k_nose_px, y + k_nose_px, ink);
    painter.ring(x, y, radius, element.line_px, ink);
}

// The orbit's path in its plane (periapsis along +x, the motion anticlockwise), as far as it
// goes for an open one, and the extent of it and the body together.
struct OrbitPath {
    std::vector<std::array<double, 2>> points_m;
    std::array<double, 2> low_m{};
    std::array<double, 2> high_m{};
};

[[nodiscard]] OrbitPath orbit_path(double semi_latus_m, double eccentricity, double limit_rad,
                                   double body_radius_m) {
    constexpr int k_points = 180;
    OrbitPath path{
        .points_m = {}, .low_m = {-body_radius_m, -body_radius_m}, .high_m = {body_radius_m, body_radius_m}};
    path.points_m.reserve(k_points + 1);
    for (int index = 0; index <= k_points; ++index) {
        const double anomaly = -limit_rad + (2.0 * limit_rad * index / k_points);
        const double radius_m = semi_latus_m / (1.0 + (eccentricity * std::cos(anomaly)));
        const std::array<double, 2> point{radius_m * std::cos(anomaly), radius_m * std::sin(anomaly)};
        path.points_m.push_back(point);
        for (std::size_t axis = 0; axis < 2; ++axis) {
            path.low_m.at(axis) = std::min(path.low_m.at(axis), point.at(axis));
            path.high_m.at(axis) = std::max(path.high_m.at(axis), point.at(axis));
        }
    }
    return path;
}

// The osculating orbit (D47), the body's centre at the focus, periapsis to the right and the
// motion anticlockwise.
void draw_orbit(Painter& painter, const ScreenElement& element, const sim::VesselView& vessel,
                const math::Vector3& colour) {
    using std::numbers::pi;
    const std::uint32_t ink = colour_of(colour);
    const double x0 = element.at_px[0];
    const double y0 = element.at_px[1];
    const double width = element.size_px[0];
    const double height = element.size_px[1];
    const auto periapsis_m = shown_value(find_signal(vessel, vessel::k_signal_periapsis));
    const auto eccentricity = shown_value(find_signal(vessel, vessel::k_signal_eccentricity));
    const auto anomaly_rad = shown_value(find_signal(vessel, vessel::k_signal_true_anomaly));
    const auto body_radius_m = shown_value(find_signal(vessel, vessel::k_signal_body_radius));
    if (!periapsis_m || !eccentricity || !anomaly_rad || !body_radius_m || *body_radius_m <= 0.0
        || *periapsis_m + *body_radius_m <= 0.0 || *eccentricity < 0.0) {
        painter.text("----", x0 + (0.5 * width), y0 + (0.5 * (height - element.height_px)), element.height_px,
                     ScreenElement::Align::Centre, ink);
        return;
    }
    const double e = *eccentricity;
    const double periapsis_radius_m = *periapsis_m + *body_radius_m;
    const double semi_latus_m = periapsis_radius_m * (1.0 + e);
    const auto radius_at = [&](double anomaly) { return semi_latus_m / (1.0 + (e * std::cos(anomaly))); };
    const bool bound = e < 1.0;
    double limit_rad = pi;
    if (!bound) {
        // Rationale: an open path has no end; a few times the periapsis, and past the vessel,
        // shows where it goes.
        constexpr double k_open_extent = 4.0;
        const double clip_m = std::max(
            {k_open_extent * periapsis_radius_m, 1.5 * radius_at(*anomaly_rad), 2.0 * *body_radius_m});
        limit_rad = std::acos(std::clamp(((semi_latus_m / clip_m) - 1.0) / e, -1.0, 1.0));
    }
    const OrbitPath path = orbit_path(semi_latus_m, e, limit_rad, *body_radius_m);
    // A margin for the apsides' marks and the letters above them.
    const double mark = 0.5 * element.height_px;
    const double margin_x =
        (0.5 * static_cast<double>(text_width("AP")) * element.height_px) + element.line_px;
    const double margin_y = mark + element.height_px + element.line_px;
    const double scale =
        std::max(0.0, std::min((width - (2.0 * margin_x)) / (path.high_m[0] - path.low_m[0]),
                               (height - (2.0 * margin_y)) / (path.high_m[1] - path.low_m[1])));
    const double middle_x = 0.5 * (path.low_m[0] + path.high_m[0]);
    const double middle_y = 0.5 * (path.low_m[1] + path.high_m[1]);
    const auto pixel = [&](const std::array<double, 2>& point) {
        return std::array{x0 + (0.5 * width) + (scale * (point[0] - middle_x)),
                          y0 + (0.5 * height) - (scale * (point[1] - middle_y))};
    };
    const auto focus = pixel({0.0, 0.0});
    painter.disc(focus[0], focus[1], scale * *body_radius_m, dimmed(colour, 0.3));
    for (std::size_t index = 1; index < path.points_m.size(); ++index) {
        const auto from = pixel(path.points_m[index - 1]);
        const auto to = pixel(path.points_m[index]);
        painter.line(from[0], from[1], to[0], to[1], element.line_px, ink);
    }
    const auto apsis = [&](const std::array<double, 2>& at, std::string_view name) {
        painter.line(at[0], at[1] - mark, at[0], at[1] + mark, element.line_px, ink);
        painter.text(name, at[0], at[1] - mark - element.height_px, element.height_px,
                     ScreenElement::Align::Centre, ink);
    };
    apsis(pixel({periapsis_radius_m, 0.0}), "PE");
    if (bound) {
        apsis(pixel({-radius_at(pi), 0.0}), "AP");
    }
    constexpr double k_vessel_px = 2.5;
    const double radius_m = radius_at(*anomaly_rad);
    const auto at = pixel({radius_m * std::cos(*anomaly_rad), radius_m * std::sin(*anomaly_rad)});
    painter.box(at[0] - k_vessel_px, at[1] - k_vessel_px, at[0] + k_vessel_px, at[1] + k_vessel_px, ink);
}

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
        if (now_s - samples[index].time_s > element.span_s || !std::isfinite(samples[index].value)
            || !std::isfinite(samples[index - 1].value)) {
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
            const std::optional<double> shown = shown_value(signal);
            const std::string value =
                shown.has_value() ? format_number(*shown * element.scale, element.decimals, element.unit)
                                  : std::string{"----"};
            const std::string words =
                element.text.empty() ? value : std::format("{} {}", element.text, value);
            painter.text(words, x0, y0, element.height_px, element.align, ink);
            break;
        }
        case ScreenElement::Type::Bar: {
            painter.frame(x0, y0, x1, y1, element.line_px, ink);
            const std::optional<double> shown = shown_value(signal);
            if (!shown.has_value()) {
                break;
            }
            const double fraction = fraction_between(*shown, element.minimum, element.maximum);
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
        case ScreenElement::Type::Attitude:
            draw_attitude(painter, element, vessel, element.colour.value_or(screen.look.accent));
            break;
        case ScreenElement::Type::Orbit:
            draw_orbit(painter, element, vessel, element.colour.value_or(screen.look.accent));
            break;
        }
    }
    return image;
}

} // namespace helios::render
