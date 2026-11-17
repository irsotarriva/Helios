#include "helios/render/flight_geometry.hpp"
#include "helios/render/screen_image.hpp"
#include "helios/render/stroke_font.hpp"
#include "helios/vessel/cockpit.hpp"
#include "helios/vessel/part_datasheet.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <gtest/gtest.h>
#include <initializer_list>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using helios::render::GraphKey;
using helios::render::ScreenImage;
using helios::render::ScreenMemory;
using helios::sim::SceneSnapshot;
using helios::sim::VesselView;
using helios::vessel::Screen;

[[nodiscard]] Screen parse_screen(const std::string& screen_toml) {
    const auto document = helios::core::parse_toml("[screen]\n" + screen_toml);
    EXPECT_TRUE(document.has_value());
    auto screen = document->at("screen").and_then(helios::vessel::parse_screen);
    EXPECT_TRUE(screen.has_value());
    return std::move(*screen);
}

// A 200 × 100 screen with two pages, chosen by `display/page`.
constexpr std::string_view k_two_pages = R"(
id = "mfd"
position_m = [0.6, 0.0, 0.0]
size_m = [0.2, 0.1]
resolution_px = [200, 100]
page_signal = "display/page"

[[screen.page]]
name = "ONE"
elements = [
  { type = "text", text = "ONE", at_px = [10, 10] },
  { type = "readout", text = "ALT", signal = "nav/altitude_m", unit = "M", decimals = 0, at_px = [10, 30] },
]

[[screen.page]]
name = "TWO"
elements = [
  { type = "bar", signal = "engine/throttle", at_px = [10, 10], size_px = [100, 20] },
  { type = "graph", signal = "nav/altitude_m", maximum = 100.0, span_s = 10.0, at_px = [0, 50], size_px = [200, 50] },
]
)";

[[nodiscard]] VesselView vessel_with(double page, double altitude_m, double throttle) {
    VesselView vessel;
    vessel.id = {0};
    vessel.signals.push_back({.name = "display/page", .command = true, .value = page, .maximum = 1.0});
    vessel.signals.push_back({.name = "nav/altitude_m", .value = altitude_m});
    vessel.signals.push_back({.name = "engine/throttle", .command = true, .value = throttle, .maximum = 1.0});
    return vessel;
}

[[nodiscard]] std::size_t text_triangles(std::string_view text) {
    return 6U * helios::render::text_strokes(text).size();
}

TEST(Screen, ReadsPagesOfElementsAndRefusesMistakes) {
    const Screen screen = parse_screen(std::string{k_two_pages});
    EXPECT_EQ(screen.resolution_px[0], 200U);
    EXPECT_EQ(screen.page_signal, "display/page");
    ASSERT_EQ(screen.pages.size(), 2U);
    EXPECT_EQ(screen.pages[0].elements.size(), 2U);
    EXPECT_EQ(screen.pages[0].elements[1].decimals, 0);
    EXPECT_EQ(screen.pages[1].elements[1].type, helios::vessel::ScreenElement::Type::Graph);
    EXPECT_EQ(helios::vessel::page_shown(screen, 0.4), 0U);
    EXPECT_EQ(helios::vessel::page_shown(screen, 0.6), 1U);
    EXPECT_EQ(helios::vessel::page_shown(screen, 7.0), 1U) << "kept within the pages";

    const auto refused = [](const std::string& screen_toml) {
        const auto document = helios::core::parse_toml("[screen]\n" + screen_toml);
        return !document.has_value() || !document->at("screen").and_then(helios::vessel::parse_screen);
    };
    const std::string place = "id = \"s\"\nposition_m = [0.6, 0.0, 0.0]\nsize_m = [0.2, 0.1]\n";
    const std::string page = "[[screen.page]]\nelements = [{ type = \"text\", text = \"A\" }]\n";
    EXPECT_FALSE(refused(place + "resolution_px = [64, 32]\n" + page));
    EXPECT_TRUE(refused(place + "resolution_px = [64, 32]\n")) << "no page";
    EXPECT_TRUE(refused(place + "resolution_px = [64.5, 32]\n" + page)) << "a fraction of a pixel";
    EXPECT_TRUE(refused(place + "resolution_px = [8, 32]\n" + page)) << "too few pixels";
    EXPECT_TRUE(
        refused(place + "resolution_px = [64, 32]\n[[screen.page]]\nelements = [{ type = \"dial\" }]\n"));
    EXPECT_TRUE(refused(place
                        + "resolution_px = [64, 32]\n[[screen.page]]\nelements = [{ type = \"bar\", "
                          "signal = \"a\", size_px = [10, 10], minimum = 1.0, maximum = 0.0 }]\n"));
    EXPECT_TRUE(refused(place
                        + "resolution_px = [64, 32]\n[[screen.page]]\nelements = [{ type = \"text\", "
                          "text = \"A\", colour = [2, 0, 0] }]\n"));
}

TEST(Screen, ShowsThePageItsSignalChooses) {
    Screen screen = parse_screen(std::string{k_two_pages});
    screen.look.accent = {0.0, 1.0, 0.0};
    const ScreenImage first = helios::render::draw_screen(screen, vessel_with(0.0, 42.0, 0.5));
    EXPECT_TRUE(first.on);
    EXPECT_EQ(first.width_px, 200U);
    EXPECT_EQ(first.triangles.size(), text_triangles("ONE") + text_triangles("ALT 42 M"));
    EXPECT_TRUE(std::ranges::all_of(first.triangles, [](const helios::render::LineVertex& vertex) {
        return vertex.abgr == helios::render::rgba(0, 255, 0) && vertex.position.z == 0.0F;
    })) << "in the screen's accent, in its pixels";

    // The second page: the bar is half full, filled from its left. The graph has no memory.
    const ScreenImage second = helios::render::draw_screen(screen, vessel_with(1.0, 42.0, 0.5));
    float right_px = 0.0F;
    for (std::size_t index = 24; index < 30; ++index) { // the fill, after the frame's four sides
        right_px = std::max(right_px, second.triangles.at(index).position.x);
    }
    EXPECT_FLOAT_EQ(right_px, 60.0F);
}

TEST(Screen, IsDarkWithoutPowerOrItsPageSignal) {
    Screen screen = parse_screen(std::string{k_two_pages});
    VesselView vessel = vessel_with(0.0, 1.0, 0.0);
    std::erase_if(vessel.signals, [](const auto& signal) { return signal.name == "display/page"; });
    const ScreenImage unpaged = helios::render::draw_screen(screen, vessel);
    EXPECT_FALSE(unpaged.on);
    EXPECT_TRUE(unpaged.triangles.empty());

    screen.page_signal.clear();
    screen.power_signal = "bus/power";
    EXPECT_FALSE(helios::render::draw_screen(screen, vessel).on) << "no such signal";
    vessel.signals.push_back({.name = "bus/power", .value = 0.2});
    EXPECT_FALSE(helios::render::draw_screen(screen, vessel).on) << "too little";
    vessel.signals.back().value = 1.0;
    EXPECT_TRUE(helios::render::draw_screen(screen, vessel).on);

    // A readout without its signal writes dashes.
    std::erase_if(vessel.signals, [](const auto& signal) { return signal.name == "nav/altitude_m"; });
    EXPECT_EQ(helios::render::draw_screen(screen, vessel).triangles.size(),
              text_triangles("ONE") + text_triangles("ALT ----"));
}

// One page with one element, 100 × 100 pixels, drawn in pure green.
[[nodiscard]] Screen one_element(std::string_view element) {
    Screen screen = parse_screen("id = \"s\"\nposition_m = [0.6, 0.0, 0.0]\nsize_m = [0.1, 0.1]\n"
                                 "resolution_px = [100, 100]\n[[screen.page]]\nelements = [ "
                                 + std::string{element} + " ]\n");
    screen.look.accent = {0.0, 1.0, 0.0};
    return screen;
}

[[nodiscard]] VesselView
vessel_signalling(std::initializer_list<std::pair<std::string_view, double>> values) {
    VesselView vessel;
    for (const auto& [name, value] : values) {
        vessel.signals.push_back({.name = std::string{name}, .value = value});
    }
    return vessel;
}

// The ground is filled at 0.3 of the accent.
constexpr std::uint32_t k_ground = helios::render::rgba(0, 77, 0);

TEST(Screen, TheAttitudeBallShowsTheHorizonAsThePilotSeesIt) {
    const Screen screen = one_element(R"({ type = "attitude", at_px = [0, 0], size_px = [100, 100] })");
    const auto ball = [&](double pitch_deg, double roll_deg) {
        constexpr double k_degree = std::numbers::pi / 180.0;
        return helios::render::draw_screen(screen,
                                           vessel_signalling({{"nav/pitch_rad", pitch_deg * k_degree},
                                                              {"nav/heading_rad", 0.0},
                                                              {"nav/roll_rad", roll_deg * k_degree}}));
    };
    const auto ground_of = [](const ScreenImage& image) {
        std::vector<helios::render::Float3> ground;
        for (const helios::render::LineVertex& vertex : image.triangles) {
            if (vertex.abgr == k_ground) {
                ground.push_back(vertex.position);
            }
        }
        return ground;
    };
    // Level: the ground is the lower half of the ball.
    const auto level = ground_of(ball(0.0, 0.0));
    ASSERT_FALSE(level.empty());
    EXPECT_TRUE(std::ranges::all_of(level, [](const auto& point) { return point.y >= 50.0F - 1e-3F; }));
    EXPECT_TRUE(std::ranges::all_of(level, [](const auto& point) {
        return std::hypot(point.x - 50.0F, point.y - 50.0F) <= 50.0F + 1e-3F;
    })) << "within the ball";
    // Rolled right, the ground comes up on the right.
    const auto rolled = ground_of(ball(0.0, 90.0));
    ASSERT_FALSE(rolled.empty());
    EXPECT_TRUE(std::ranges::all_of(rolled, [](const auto& point) { return point.x >= 50.0F - 1e-3F; }));
    // Nose up 30 degrees, the horizon ahead is half a radius below the centre.
    const auto climbing = ground_of(ball(30.0, 0.0));
    float highest_ahead = 100.0F;
    for (const auto& point : climbing) {
        if (std::abs(point.x - 50.0F) < 1.0F) {
            highest_ahead = std::min(highest_ahead, point.y);
        }
    }
    EXPECT_NEAR(highest_ahead, 75.0F, 0.5F);

    // Without its signals, a ring and dashes.
    EXPECT_TRUE(ground_of(helios::render::draw_screen(screen, vessel_signalling({}))).empty());
}

TEST(Screen, TheOrbitPlotFitsTheOrbitWithItsBodyAtTheFocus) {
    const Screen screen =
        one_element(R"({ type = "orbit", at_px = [0, 0], size_px = [100, 100], height_px = 6 })");
    const auto plot = [&](double eccentricity, double anomaly_rad) {
        return helios::render::draw_screen(screen, vessel_signalling({{"nav/periapsis_m", 200e3},
                                                                      {"nav/eccentricity", eccentricity},
                                                                      {"nav/true_anomaly_rad", anomaly_rad},
                                                                      {"nav/body_radius_m", 1'000e3}}));
    };
    const auto inside = [](const ScreenImage& image) {
        return std::ranges::all_of(image.triangles, [](const helios::render::LineVertex& vertex) {
            return vertex.position.x >= -1.0F && vertex.position.x <= 101.0F && vertex.position.y >= -1.0F
                   && vertex.position.y <= 101.0F;
        });
    };
    // The body is drawn first: its disc's first corner is the focus.
    const ScreenImage circle = plot(0.0, 0.0);
    ASSERT_FALSE(circle.triangles.empty());
    EXPECT_EQ(circle.triangles[0].abgr, k_ground);
    EXPECT_NEAR(circle.triangles[0].position.x, 50.0F, 1e-3F);
    EXPECT_NEAR(circle.triangles[0].position.y, 50.0F, 1e-3F);
    EXPECT_TRUE(inside(circle));
    // An ellipse reaches further on the apoapsis side, to the left: the focus is right of centre.
    const ScreenImage ellipse = plot(0.5, 1.0);
    EXPECT_GT(ellipse.triangles[0].position.x, 55.0F);
    EXPECT_TRUE(inside(ellipse));
    const ScreenImage hyperbola = plot(1.5, -0.5);
    EXPECT_TRUE(inside(hyperbola));
    EXPECT_FALSE(hyperbola.triangles.empty());

    // No orbit (NaN), dashes.
    const ScreenImage none = plot(std::numeric_limits<double>::quiet_NaN(), 0.0);
    EXPECT_EQ(none.triangles.size(), text_triangles("----"));
}

// A snapshot of a vessel whose one part has a cockpit with the two-page screen.
[[nodiscard]] SceneSnapshot snapshot_at(double seconds, double altitude_m) {
    helios::vessel::PartDatasheet pod;
    pod.id = "test.pod";
    helios::vessel::Cockpit cockpit;
    cockpit.eye_m = {0.0, 0.0, 0.0};
    cockpit.screens.push_back(parse_screen(std::string{k_two_pages}));
    EXPECT_TRUE(helios::vessel::resolve_looks(cockpit, {}).has_value());
    pod.cockpit = std::move(cockpit);
    SceneSnapshot snapshot;
    snapshot.epoch = helios::time::Epoch::from_seconds(seconds).value();
    VesselView vessel = vessel_with(1.0, altitude_m, 0.0);
    vessel.parts.resize(1);
    vessel.parts[0].datasheet = std::make_shared<const helios::vessel::PartDatasheet>(std::move(pod));
    snapshot.vessels.push_back(std::move(vessel));
    return snapshot;
}

TEST(Screen, GraphsRememberTheirSignalsOverTheirSpan) {
    ScreenMemory memory;
    const GraphKey graph{.vessel = {0}, .part = 0, .screen = 0, .page = 1, .element = 1};
    for (int second = 0; second <= 30; ++second) {
        memory.record(snapshot_at(100.0 + second, second));
    }
    const auto samples = memory.samples(graph);
    ASSERT_FALSE(samples.empty());
    EXPECT_DOUBLE_EQ(samples.back().value, 30.0);
    EXPECT_DOUBLE_EQ(memory.now_s(), 30.0);
    // The span (10 s) and one sample before it, for the trace to reach the edge.
    EXPECT_DOUBLE_EQ(samples.front().time_s, 19.0);
    EXPECT_EQ(samples.size(), 12U);
    EXPECT_TRUE(memory.samples({.vessel = {0}, .part = 0, .screen = 0, .page = 0, .element = 0}).empty())
        << "only graphs";

    // Drawn, the trace is a line between each two samples, the oldest clipped at the left edge.
    const SceneSnapshot now = snapshot_at(130.0, 30.0);
    const std::optional<helios::vessel::Cockpit>& cockpit = now.vessels[0].parts[0].datasheet->cockpit;
    ASSERT_TRUE(cockpit.has_value());
    const ScreenImage image = helios::render::draw_screen(cockpit->screens[0], now.vessels[0], &memory);
    constexpr std::size_t k_quad = 6;   // two triangles
    const std::size_t bar = 5 * k_quad; // its frame's four sides, and a fill of nothing
    const std::size_t frame = 4 * k_quad;
    EXPECT_EQ(image.triangles.size(), bar + frame + (11 * k_quad));

    // Time going back (a scenario loaded again) forgets.
    memory.record(snapshot_at(50.0, 1.0));
    EXPECT_EQ(memory.samples(graph).size(), 1U);
}

TEST(Screen, StandsInTheCockpitSeenFromTheSeat) {
    SceneSnapshot snapshot = snapshot_at(0.0, 1.0);
    snapshot.focus = helios::sim::Focus::vessel({0});
    const helios::render::ViewPoint seat =
        helios::render::seat_view_point(snapshot.vessels[0], helios::render::HeadPose{}).value();
    const helios::render::FlightGeometry geometry =
        helios::render::build_flight_geometry(snapshot, 0, seat, true);
    ASSERT_EQ(geometry.screens.size(), 1U);
    const helios::render::ScreenSurface& surface = geometry.screens[0];
    EXPECT_EQ(surface.screen, 0U);
    EXPECT_TRUE(surface.image.on);
    // The glass is 0.2 m across and 0.1 m up, its top left to the pilot's left and above.
    const auto distance = [](const helios::render::Float3& a, const helios::render::Float3& b) {
        return std::hypot(a.x - b.x, a.y - b.y, a.z - b.z);
    };
    EXPECT_NEAR(distance(surface.corners[0], surface.corners[1]), 0.2F, 1e-5F);
    EXPECT_NEAR(distance(surface.corners[1], surface.corners[2]), 0.1F, 1e-5F);
    EXPECT_FALSE(geometry.solids.empty()) << "the bezel";
    EXPECT_TRUE(helios::render::build_flight_geometry(snapshot, 0, seat, false).screens.empty())
        << "not from outside";
}

} // namespace
