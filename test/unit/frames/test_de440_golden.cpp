// Regression test against JPL DE440: the frame tree built from a three-month excerpt of the
// exported ephemeris must reproduce states computed independently by jplephem.
// Golden values: tools/ephemeris/golden_states.py (jplephem 2.24 on JPL's de440.bsp), J2000
// ecliptic, SI units.

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
    GoldenState{"Moon-Earth", 631238400, 0.0, {401920637.3204552, 25760490.56351001, -37149987.08552944}, {-69.0044577524909, 963.1714943304437, -6.292504674972492}},
    GoldenState{"Moon-Earth", 632275200, 0.25, {-348948593.7024653, 106788025.64366163, 30430540.69459292}, {-317.90029291306615, -1023.2027023477748, 42.25792427687576}},
    GoldenState{"Moon-Earth", 633657600, 0.5, {395215327.55097747, 81464969.69078313, -36621637.48973618}, {-202.42562147923175, 944.7060632078427, 6.830978824701944}},
    GoldenState{"Moon-Earth", 635212800, 0.75, {-83202259.11833346, -374060129.07035476, 11510486.078194203}, {981.3081054822342, -273.781349981986, -84.55551394772269}},
    GoldenState{"Earth-Sun", 631238400, 0.0, {-28743979137.136658, 144258581858.5985, -6096292.84036077}, {-29700.172308821508, -5944.27776865438, 0.42303510923276555}},
    GoldenState{"Earth-Sun", 632275200, 0.25, {-58657719632.68541, 134931683007.29633, -6413830.601096577}, {-27801.225989115195, -11976.754376979708, 0.10778650550932932}},
    GoldenState{"Earth-Sun", 633657600, 0.5, {-94280079091.16785, 113246585654.44777, -4511510.133115886}, {-23378.002559099972, -19182.46561379822, 0.8585274761628652}},
    GoldenState{"Earth-Sun", 635212800, 0.75, {-125356078698.43358, 78313105138.57147, -3409903.7935386077}, {-16280.568894683216, -25373.392214272004, 2.235534034310576}},
    GoldenState{"Mars-Sun", 631238400, 0.0, {-195601786123.29156, -134835215100.58476, 1973809778.966379}, {14658.898330030353, -17875.421065435745, -734.2357258393184}},
    GoldenState{"Mars-Sun", 632275200, 0.25, {-179382224018.38617, -152612473822.87863, 1203341257.0501556}, {16609.21751120279, -16379.810385104274, -750.7440100077804}},
    GoldenState{"Mars-Sun", 633657600, 0.5, {-154737447020.77054, -173684342893.26794, 157126275.55942088}, {19001.909977005733, -14041.344644961797, -760.4419727045806}},
    GoldenState{"Mars-Sun", 635212800, 0.75, {-123315906313.10997, -193184285992.78305, -1022408974.7229162}, {21335.931475240734, -10958.369799216493, -753.0987880165701}},
};
// clang-format on

[[nodiscard]] FrameTree load_excerpt() {
    const std::filesystem::path path = std::filesystem::path{HELIOS_TEST_DATA_DIR} / "de440_2020_excerpt.hce";
    auto segments = helios::ephemeris::load_chebyshev_file(path).value();
    FrameTree tree(segments.front().center_name);
    for (auto& segment : segments) {
        const auto center = tree.find(segment.center_name).value();
        static_cast<void>(tree.add_frame(segment.target_name, center, std::move(segment.ephemeris)).value());
    }
    return tree;
}

TEST(De440Golden, FrameTreeReproducesJplStatesToAMillimetre) {
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

TEST(De440Golden, ExcerptCoverageIsEnforced) {
    const FrameTree tree = load_excerpt();
    const auto moon = tree.find("Moon").value();
    const auto earth = tree.find("Earth").value();
    const Epoch far_future = Epoch::from_parts(2'000'000'000, 0.0).value();
    EXPECT_EQ(tree.relative_state(moon, earth, far_future).error().code, helios::core::ErrorCode::OutOfRange);
}

} // namespace
