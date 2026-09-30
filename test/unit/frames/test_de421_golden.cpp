// Regression test against JPL DE421: the frame tree built from a two-month excerpt of the
// exported ephemeris must reproduce states computed independently by jplephem.
// Golden values: tools/ephemeris (jplephem 2.x on de421.bsp), J2000 ecliptic, SI units.

#include "helios/ephemeris/ephemeris_file.hpp"
#include "helios/frames/frame_tree.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <gtest/gtest.h>
#include <string_view>

namespace {

using helios::frames::FrameTree;
using helios::math::norm;
using helios::math::Vector3;
using helios::time::Epoch;

struct GoldenState {
    std::string_view pair;
    std::int64_t whole_seconds;
    double fraction_s;
    Vector3 position_m;
    Vector3 velocity_m_s;
};

// clang-format off
constexpr std::array k_golden_states{
    GoldenState{"Moon-Earth", 631238400, 0.0, {401920637.7122216, 25760488.228809662, -37149986.63904031}, {-69.00445179822445, 963.1714953936159, -6.29250551929226}},
    GoldenState{"Moon-Earth", 632275200, 0.25, {-348948593.21226925, 106788028.1955284, 30430540.197201386}, {-317.9003000698759, -1023.2027007165058, 42.25792476695973}},
    GoldenState{"Moon-Earth", 633657600, 0.5, {395215328.2571286, 81464967.43653266, -36621637.09358}, {-202.4256157973664, 944.7060651239151, 6.830977857077079}},
    GoldenState{"Moon-Earth", 635212800, 0.75, {-83202261.70313358, -374060128.594779, 11510486.28586936}, {981.3081045524153, -273.78135689731516, -84.55551261911008}},
    GoldenState{"Earth-Sun", 631238400, 0.0, {-28743979074.97189, 144258581869.60187, -6096357.078578577}, {-29700.17231167854, -5944.277755980829, 0.4229949244915206}},
    GoldenState{"Earth-Sun", 632275200, 0.25, {-58657719574.82918, 134931683030.85535, -6413934.75068083}, {-27801.22599438655, -11976.754365469971, 0.10774995904476956}},
    GoldenState{"Earth-Sun", 633657600, 0.5, {-94280079042.70348, 113246585692.88928, -4511660.043877049}, {-23378.002567503165, -19182.465604055524, 0.8584982906775217}},
    GoldenState{"Earth-Sun", 635212800, 0.75, {-125356078664.99881, 78313105189.53314, -3410090.9247245532}, {-16280.568905337414, -25373.39220779655, 2.2355157066456015}},
    GoldenState{"Mars-Sun", 631238400, 0.0, {-195601786303.08298, -134835214855.909, 1973809525.687271}, {14658.898304401386, -17875.421087418217, -734.2357015352522}},
    GoldenState{"Mars-Sun", 632275200, 0.25, {-179382224223.83426, -152612473602.31604, 1203341030.2876804}, {16609.21748736737, -16379.810409626936, -750.7439831952332}},
    GoldenState{"Mars-Sun", 633657600, 0.5, {-154737447257.37006, -173684342708.90213, 157126087.97956258}, {19001.909955824496, -14041.344672800686, -760.4419429038446}},
    GoldenState{"Mars-Sun", 635212800, 0.75, {-123315906580.03088, -193184285854.61966, -1022409113.7236885}, {21335.931457554812, -10958.369830793907, -753.0987554653079}},
};
// clang-format on

[[nodiscard]] FrameTree load_excerpt() {
    const std::filesystem::path path = std::filesystem::path{HELIOS_TEST_DATA_DIR} / "de421_2020_excerpt.hce";
    auto segments = helios::ephemeris::load_chebyshev_file(path).value();
    FrameTree tree(segments.front().center_name);
    for (auto& segment : segments) {
        const auto center = tree.find(segment.center_name).value();
        static_cast<void>(tree.add_frame(segment.target_name, center, std::move(segment.ephemeris)).value());
    }
    return tree;
}

TEST(De421Golden, FrameTreeReproducesJplStatesToAMillimetre) {
    const FrameTree tree = load_excerpt();
    for (const GoldenState& golden : k_golden_states) {
        const std::string_view target_name = golden.pair.substr(0, golden.pair.find('-'));
        const std::string_view observer_name = golden.pair.substr(golden.pair.find('-') + 1);
        const auto target = tree.find(target_name).value();
        const auto observer = tree.find(observer_name).value();
        const Epoch instant = Epoch::from_parts(golden.whole_seconds, golden.fraction_s).value();

        const auto state = tree.relative_state(target, observer, instant);
        ASSERT_TRUE(state.has_value()) << golden.pair;
        EXPECT_LT(norm(state->position_m - golden.position_m), 1e-3)
            << golden.pair << " @ " << golden.whole_seconds;
        EXPECT_LT(norm(state->velocity_m_s - golden.velocity_m_s), 1e-9)
            << golden.pair << " @ " << golden.whole_seconds;
    }
}

TEST(De421Golden, ExcerptCoverageIsEnforced) {
    const FrameTree tree = load_excerpt();
    const auto moon = tree.find("Moon").value();
    const auto earth = tree.find("Earth").value();
    const Epoch far_future = Epoch::from_parts(2'000'000'000, 0.0).value();
    EXPECT_EQ(tree.relative_state(moon, earth, far_future).error().code, helios::core::ErrorCode::OutOfRange);
}

} // namespace
