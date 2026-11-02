#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

#include "vr_hands.hpp"

namespace {

using helios::app::Controller;
using helios::app::Press;
using helios::app::SnapTurn;
using helios::app::Squeeze;
using helios::math::Matrix3;
using helios::math::norm;
using helios::math::Quaternion;
using helios::math::Vector3;

constexpr double k_quarter_turn_rad = 0.5 * std::numbers::pi;

void expect_near(const Vector3& actual, const Vector3& expected, double tolerance = 1e-9) {
    EXPECT_NEAR(norm(actual - expected), 0.0, tolerance)
        << "(" << actual.x << ", " << actual.y << ", " << actual.z << ")";
}

TEST(VrHands, AGripClosesFirmlyAndOpensFirmly) {
    Squeeze grip;
    EXPECT_FALSE(grip.update(0.5));
    EXPECT_FALSE(grip.closed());
    EXPECT_TRUE(grip.update(0.7)); // closes, once
    EXPECT_FALSE(grip.update(0.9));
    EXPECT_TRUE(grip.closed());
    EXPECT_FALSE(grip.update(0.5)); // loosened, still held
    EXPECT_TRUE(grip.closed());
    EXPECT_FALSE(grip.update(0.3));
    EXPECT_FALSE(grip.closed());
}

TEST(VrHands, AButtonIsPressedOnceWhileHeld) {
    Press button;
    EXPECT_TRUE(button.update(true));
    EXPECT_FALSE(button.update(true));
    EXPECT_TRUE(button.down());
    EXPECT_FALSE(button.update(false));
    EXPECT_TRUE(button.update(true));
}

TEST(VrHands, TheThumbstickTurnsInStepsAndMustComeBackBetweenThem) {
    SnapTurn turn;
    EXPECT_DOUBLE_EQ(turn.update(0.5), 0.0);                   // not far enough
    EXPECT_DOUBLE_EQ(turn.update(0.8), -SnapTurn::k_step_rad); // to the right
    EXPECT_DOUBLE_EQ(turn.update(1.0), 0.0);                   // held over: once
    EXPECT_DOUBLE_EQ(turn.update(0.5), 0.0);
    EXPECT_DOUBLE_EQ(turn.update(0.9), 0.0); // not back in the middle yet
    EXPECT_DOUBLE_EQ(turn.update(0.1), 0.0);
    EXPECT_DOUBLE_EQ(turn.update(-0.8), SnapTurn::k_step_rad); // to the left
}

TEST(VrHands, TheFingertipsReachOutAlongTheHand) {
    // A hand held at the side, pointing to the left.
    const Controller hand{.tracked = true,
                          .position_m = {0.3, 0.2, -0.3},
                          .orientation = helios::math::from_axis_angle({0.0, 0.0, 1.0}, k_quarter_turn_rad)};
    expect_near(helios::app::fingertip_m(hand), {0.3, 0.2 + helios::app::k_fingertip_m, -0.3});
}

TEST(VrHands, TheHeadsAxesBecomeCameraSpace) {
    // The head turned a quarter to the left: what is to its right is straight ahead in tracking.
    const Quaternion head = helios::math::from_axis_angle({0.0, 0.0, 1.0}, k_quarter_turn_rad);
    expect_near(helios::app::in_head_axes({1.0, 0.0, 0.0}, head), {0.0, -1.0, 0.0});
    // A camera looking down the universe's −z with its up along +y (rows: right, up, back).
    const Matrix3 universe_to_camera{};
    expect_near(helios::app::head_to_camera_space({1.0, 0.0, 0.0}, universe_to_camera), {0.0, 0.0, -1.0});
    expect_near(helios::app::head_to_camera_space({0.0, 1.0, 0.0}, universe_to_camera), {-1.0, 0.0, 0.0});
    expect_near(helios::app::head_to_camera_space({0.0, 0.0, 1.0}, universe_to_camera), {0.0, 1.0, 0.0});
    // The hand's axes as columns: a hand like the head points where the camera looks.
    const Matrix3 hand = helios::app::hand_to_camera_space(Quaternion{}, universe_to_camera);
    expect_near({hand(0, 0), hand(1, 0), hand(2, 0)}, {0.0, 0.0, -1.0});
    expect_near({hand(0, 2), hand(1, 2), hand(2, 2)}, {0.0, 1.0, 0.0});
}

TEST(VrHands, AHandThatHoldsOnMovesTheBodyTheOtherWay) {
    // Pulling the hand in by 5 cm over a tenth of a second draws the body forward at half a metre
    // a second.
    expect_near(helios::app::arm_from_hand({-0.05, 0.0, 0.0}, 0.1), {0.5, 0.0, 0.0});
    expect_near(helios::app::arm_from_hand({1.0, 0.0, 0.0}, 0.0), {});
}

TEST(VrHands, TheThumbstickWalks) {
    const Controller stick{.stick_x = 0.5, .stick_y = 1.0};
    expect_near(helios::app::stick_to_move(stick), {1.0, -0.5, 0.0});
}

} // namespace
