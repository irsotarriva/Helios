#include "helios/render/star_field.hpp"

#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>

namespace {

using helios::math::Vector3;
using helios::render::Star;

// The galactic pole, as star_field.cpp has it in ecliptic axes.
const Vector3 k_galactic_pole =
    Vector3{-0.8677, -0.0003, 0.4970} / helios::math::norm({-0.8677, -0.0003, 0.4970});

TEST(StarField, IsTheSameSkyEveryTime) {
    const std::vector<Star> first = helios::render::make_star_field();
    const std::vector<Star> again = helios::render::make_star_field();
    ASSERT_EQ(first.size(), helios::render::k_star_count);
    ASSERT_EQ(again.size(), first.size());
    for (std::size_t index = 0; index < first.size(); ++index) {
        EXPECT_EQ(first[index].direction, again[index].direction);
        EXPECT_EQ(first[index].abgr, again[index].abgr);
    }
    EXPECT_NE(helios::render::make_star_field(10, 2)[0].direction, first[0].direction) << "another seed";
}

TEST(StarField, HasFewBrightStarsManyFaintOnesAndCrowdsAlongTheMilkyWay) {
    const std::vector<Star> stars = helios::render::make_star_field();
    std::size_t bright = 0;
    std::size_t faint = 0;
    std::size_t in_the_band = 0;
    for (const Star& star : stars) {
        EXPECT_NEAR(helios::math::norm(star.direction), 1.0, 1e-12);
        EXPECT_GE(star.brightness, 0.3F);
        EXPECT_LE(star.brightness, 1.0F);
        EXPECT_EQ(star.abgr >> 24U, 255U) << "opaque";
        bright += star.brightness > 0.9F ? 1U : 0U;
        faint += star.brightness < 0.35F ? 1U : 0U;
        // Within 15° of the galactic plane: a quarter of an even sky.
        in_the_band +=
            std::abs(helios::math::dot(star.direction, k_galactic_pole)) < std::sin(0.2618) ? 1U : 0U;
    }
    EXPECT_GT(bright, 0U);
    EXPECT_LT(bright, stars.size() / 20);
    EXPECT_GT(faint, stars.size() / 2);
    EXPECT_GT(static_cast<double>(in_the_band) / static_cast<double>(stars.size()), 0.4);
}

TEST(StarField, EachStarIsAShortLineFarAwayAlongItsDirection) {
    const std::vector<Star> stars = helios::render::make_star_field(50);
    const std::vector<helios::render::LineVertex> lines = helios::render::star_lines(stars);
    ASSERT_EQ(lines.size(), 2 * stars.size());
    for (std::size_t index = 0; index < stars.size(); ++index) {
        const auto& from = lines[2 * index].position;
        const auto& to = lines[(2 * index) + 1].position;
        const Vector3 a{from.x, from.y, from.z};
        const Vector3 b{to.x, to.y, to.z};
        const Vector3 middle = 0.5 * (a + b);
        EXPECT_NEAR(helios::math::norm(middle) / helios::render::k_star_distance_m, 1.0, 1e-6);
        EXPECT_NEAR(helios::math::dot(middle / helios::math::norm(middle), stars[index].direction), 1.0,
                    1e-9);
        EXPECT_NEAR(helios::math::norm(b - a) / helios::render::k_star_distance_m,
                    helios::render::k_star_length_rad, 1e-6);
        EXPECT_EQ(lines[2 * index].abgr, stars[index].abgr);
    }
}

} // namespace
