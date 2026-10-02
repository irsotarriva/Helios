#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

#include "pilot_controls.hpp"

namespace {

using helios::app::look_from_seat;
using helios::app::PilotLook;
using helios::app::turn;
using helios::app::view_of;
using helios::math::dot;
using helios::math::Matrix3;
using helios::math::norm;
using helios::math::rotate;
using helios::math::Vector3;

constexpr double k_quarter_turn_rad = 0.5 * std::numbers::pi;
constexpr Vector3 k_forward{1.0, 0.0, 0.0};
constexpr Vector3 k_left{0.0, 1.0, 0.0};
constexpr Vector3 k_up{0.0, 0.0, 1.0};

void expect_near(const Vector3& actual, const Vector3& expected, double tolerance = 1e-9) {
    EXPECT_NEAR(norm(actual - expected), 0.0, tolerance)
        << "(" << actual.x << ", " << actual.y << ", " << actual.z << ")";
}

TEST(PilotLook, StartsFromTheSeatWithTheHeadAsItWas) {
    // A seat that faces the vessel's +z with the vessel's nose (+x) overhead.
    const Matrix3 sideways{{0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0}};
    const PilotLook ahead = look_from_seat(sideways, 0.0, 0.0);
    expect_near(rotate(view_of(ahead), k_forward), {0.0, 0.0, 1.0});
    expect_near(rotate(view_of(ahead), k_up), {1.0, 0.0, 0.0});
    // Head turned a quarter to the left and tipped up: left of that seat is the vessel's −y.
    const PilotLook turned = look_from_seat(sideways, k_quarter_turn_rad, 0.3);
    expect_near(rotate(turned.body, k_forward), {0.0, -1.0, 0.0});
    EXPECT_DOUBLE_EQ(turned.pitch_rad, 0.3);
    EXPECT_NEAR(rotate(view_of(turned), k_forward).x, std::sin(0.3), 1e-12); // towards the nose
}

TEST(PilotLook, AFloatingBodyTurnsFreelyAboutItsOwnAxes) {
    PilotLook look;
    turn(look, k_quarter_turn_rad, 0.0, 0.0, {}, 0.02);
    expect_near(rotate(view_of(look), k_forward), k_left);
    // Up, past the vertical and on: nothing stops a body in mid-air from going head over heels.
    turn(look, 0.0, 2.0 * k_quarter_turn_rad, 0.0, {}, 0.02);
    expect_near(rotate(view_of(look), k_forward), -1.0 * k_left);
    expect_near(rotate(view_of(look), k_up), -1.0 * k_up);
    EXPECT_EQ(look.pitch_rad, 0.0);
    // Rolling to the right takes the head towards the right-hand side.
    PilotLook rolled;
    turn(rolled, 0.0, 0.0, k_quarter_turn_rad, {}, 0.02);
    expect_near(rotate(view_of(rolled), k_forward), k_forward);
    expect_near(rotate(view_of(rolled), k_up), -1.0 * k_left);
}

TEST(PilotLook, WithWeightTheBodyRightsItselfAndOnlyTheHeadPitches) {
    // The weight pulls towards the vessel's −x; the body starts with its head towards +z.
    const Vector3 up{1.0, 0.0, 0.0};
    PilotLook look;
    turn(look, 0.0, 0.0, 0.0, up, 0.1);
    // 4 rad/s for a tenth of a second: 0.4 rad of the quarter turn there is to make.
    EXPECT_NEAR(std::acos(rotate(look.body, k_up).x), k_quarter_turn_rad - 0.4, 1e-9);
    for (int frame = 0; frame < 60; ++frame) {
        turn(look, 0.0, 0.0, 0.5, up, 1.0 / 60.0); // the roll asked for is not given
    }
    expect_near(rotate(look.body, k_up), up);
    EXPECT_NEAR(rotate(view_of(look), k_forward).x, 0.0, 1e-9); // looking level

    // The head pitches, as far as a neck goes; the body stays upright.
    turn(look, 0.0, 10.0, 0.0, up, 0.02);
    EXPECT_EQ(look.pitch_rad, PilotLook::k_max_pitch_rad);
    expect_near(rotate(look.body, k_up), up);
    EXPECT_NEAR(rotate(view_of(look), k_forward).x, std::sin(PilotLook::k_max_pitch_rad), 1e-9);
    // Turning is about the body's own up.
    const Vector3 before = rotate(look.body, k_forward);
    turn(look, k_quarter_turn_rad, 0.0, 0.0, up, 0.02);
    EXPECT_NEAR(dot(rotate(look.body, k_forward), before), 0.0, 1e-9);
    expect_near(rotate(look.body, k_up), up);

    // The weight goes (the engine stops): the view stays where it was, and the pitch is now
    // the body's.
    const Vector3 gaze = rotate(view_of(look), k_forward);
    turn(look, 0.0, 0.0, 0.0, {}, 0.02);
    expect_near(rotate(view_of(look), k_forward), gaze);
    EXPECT_EQ(look.pitch_rad, 0.0);

    // Exactly upside down there is still a way round.
    PilotLook inverted;
    turn(inverted, 0.0, 0.0, 0.0, -1.0 * k_up, 0.1);
    EXPECT_NEAR(std::acos(rotate(inverted.body, k_up).z), 0.4, 1e-9);
}

} // namespace
