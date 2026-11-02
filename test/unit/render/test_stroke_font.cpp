#include "helios/render/stroke_font.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

namespace {

using helios::render::TextStroke;

[[nodiscard]] std::string printable() {
    std::string all;
    for (char letter = ' '; letter <= '~'; ++letter) {
        all.push_back(letter);
    }
    return all;
}

TEST(StrokeFont, EveryLetterStaysInItsCell) {
    const std::string all = printable();
    constexpr float k_slack = 1e-5F;
    for (const char character : all) {
        const std::string letter(1, character);
        const std::vector<TextStroke> strokes = helios::render::text_strokes(letter);
        if (letter != " ") {
            EXPECT_FALSE(strokes.empty()) << letter;
        }
        for (const TextStroke& stroke : strokes) {
            for (const auto [x, y] : {std::pair{stroke.x0, stroke.y0}, std::pair{stroke.x1, stroke.y1}}) {
                EXPECT_GE(x, -k_slack) << letter;
                EXPECT_LE(x, helios::render::k_letter_width + k_slack) << letter;
                EXPECT_GE(y, (-2.0F / 7.0F) - k_slack) << letter; // the descenders
                EXPECT_LE(y, 1.0F + k_slack) << letter;
            }
        }
    }
}

TEST(StrokeFont, LettersFollowOneAnotherAndUnknownOnesAreQuestionMarks) {
    EXPECT_EQ(helios::render::text_width(""), 0.0F);
    EXPECT_FLOAT_EQ(helios::render::text_width("A"), helios::render::k_letter_width);
    EXPECT_FLOAT_EQ(helios::render::text_width("ALT"),
                    (2.0F * helios::render::k_text_advance) + helios::render::k_letter_width);
    // The second letter of "II" is the first moved along by one advance.
    const std::vector<TextStroke> one = helios::render::text_strokes("I");
    const std::vector<TextStroke> two = helios::render::text_strokes("II");
    ASSERT_EQ(two.size(), 2U * one.size());
    for (std::size_t index = 0; index < one.size(); ++index) {
        EXPECT_FLOAT_EQ(two[one.size() + index].x0, one[index].x0 + helios::render::k_text_advance);
        EXPECT_FLOAT_EQ(two[one.size() + index].y1, one[index].y1);
    }
    // A byte of UTF-8 is drawn as '?'.
    EXPECT_EQ(helios::render::text_strokes("\xC2").size(), helios::render::text_strokes("?").size());
}

TEST(StrokeFont, DigitsAndLettersCanBeToldApart) {
    const std::string all = printable();
    for (std::size_t first = 0; first < all.size(); ++first) {
        for (std::size_t second = first + 1; second < all.size(); ++second) {
            const auto a = helios::render::text_strokes(std::string(1, all[first]));
            const auto b = helios::render::text_strokes(std::string(1, all[second]));
            const bool same =
                a.size() == b.size() && std::ranges::equal(a, b, [](const auto& p, const auto& q) {
                    return p.x0 == q.x0 && p.y0 == q.y0 && p.x1 == q.x1 && p.y1 == q.y1;
                });
            // Only the letters without a shape of their own are drawn as '?'.
            const std::string unshaped = "&?@{}";
            const bool both_unshaped = unshaped.contains(all[first]) && unshaped.contains(all[second]);
            EXPECT_TRUE(!same || both_unshaped) << all[first] << " and " << all[second];
        }
    }
}

TEST(StrokeFont, TextIsLaidCentredInItsPlane) {
    std::vector<helios::render::LineVertex> triangles;
    // On a panel facing +x, 2 m ahead, with letters 1 cm high: right is -y, up is +z.
    const helios::render::TextPlacement placement{
        .centre_m = {2.0, 0.0, 0.0}, .right_m = {0.0, -0.01, 0.0}, .up_m = {0.0, 0.0, 0.01}};
    helios::render::append_text(triangles, "HOLD", placement, 0xFF112233U);
    ASSERT_EQ(triangles.size(), 6U * helios::render::text_strokes("HOLD").size());
    float least_y = 1.0F;
    float most_y = -1.0F;
    float least_z = 1.0F;
    float most_z = -1.0F;
    for (const helios::render::LineVertex& vertex : triangles) {
        EXPECT_EQ(vertex.abgr, 0xFF112233U);
        EXPECT_FLOAT_EQ(vertex.position.x, 2.0F);
        least_y = std::min(least_y, vertex.position.y);
        most_y = std::max(most_y, vertex.position.y);
        least_z = std::min(least_z, vertex.position.z);
        most_z = std::max(most_z, vertex.position.z);
    }
    // As wide as the text and as high as a capital (no descenders), plus half a stroke each side,
    // or a little more where a slanted stroke ends (its corners stand out by up to √2 times).
    const float half_stroke_m = 0.5F * helios::render::k_stroke_width * 0.01F;
    const float half_width_m = 0.5F * helios::render::text_width("HOLD") * 0.01F;
    const auto beyond = [&](float extent, float edge) {
        EXPECT_GE(extent, edge + half_stroke_m - 1e-6F);
        EXPECT_LE(extent, edge + (std::numbers::sqrt2_v<float> * half_stroke_m) + 1e-6F);
    };
    beyond(most_y, half_width_m);
    beyond(-least_y, half_width_m);
    beyond(most_z, 0.005F);
    beyond(-least_z, 0.005F);
    // H starts at the left, which is +y here.
    EXPECT_GT(triangles.front().position.y, 0.0F);
}

} // namespace
