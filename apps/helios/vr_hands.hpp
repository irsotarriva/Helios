#ifndef HELIOS_APPS_HELIOS_VR_HANDS_HPP
#define HELIOS_APPS_HELIOS_VR_HANDS_HPP

#include "helios/math/matrix3.hpp"
#include "helios/math/quaternion.hpp"
#include "helios/math/vector3.hpp"

// The hands of a headset's controllers (BRIEFING D39): they drive what the mouse does on a flat
// screen. The trigger handles the control the fingertips touch (presses a button or a switch,
// holds a lever while it is pulled); out of the seat the grip holds on (the right hand is the
// grabbing hand of D27, the left the latching one) and the hand's own movement is the arm's.
namespace helios::app {

// A controller as the client reads it. Positions and turns are in the headset's tracking axes
// (forward, left, up), relative to the seat's eye point.
struct Controller {
    bool tracked = false;
    math::Vector3 position_m;     // where the hand holds it
    math::Quaternion orientation; // takes the hand's axes (forward, left, up) to the tracking axes
    double squeeze = 0.0;         // the grip, 0 to 1
    double trigger = 0.0;         // 0 to 1
    double stick_x = 0.0;         // the thumbstick: to the right, -1 to 1
    double stick_y = 0.0;         // forward, -1 to 1
    bool primary = false;         // A or X
    bool secondary = false;       // B or Y
};

// From where the hand holds a controller to the tips of the fingers reaching out of it.
inline constexpr double k_fingertip_m = 0.08;
// How near the fingertips are to a control's handle to touch it, beyond its own radius.
inline constexpr double k_touch_reach_m = 0.03;

// A pull of a grip or a trigger read as closed or open: it closes past 0.6 and opens below 0.4,
// so a finger resting halfway does not make it flicker.
class Squeeze {
public:
    static constexpr double k_close = 0.6;
    static constexpr double k_open = 0.4;

    // True on the frame it closes.
    bool update(double value) noexcept;
    [[nodiscard]] bool closed() const noexcept { return closed_; }

private:
    bool closed_ = false;
};

// A button read for the frame it goes down.
class Press {
public:
    // True on the frame it goes down.
    bool update(bool down) noexcept;
    [[nodiscard]] bool down() const noexcept { return down_; }

private:
    bool down_ = false;
};

// Turning by the thumbstick in steps, which is easier on the stomach than a smooth turn: a push
// past 0.7 to one side turns by k_step_rad once, and the stick must come back under 0.3 first.
class SnapTurn {
public:
    static constexpr double k_step_rad = 0.5235987755982988; // 30°
    static constexpr double k_push = 0.7;
    static constexpr double k_centred = 0.3;

    // The turn to the left this frame, in radians (negative: to the right).
    double update(double stick_x) noexcept;

private:
    bool centred_ = true;
};

// The fingertips, in the tracking axes.
[[nodiscard]] math::Vector3 fingertip_m(const Controller& controller) noexcept;

// A direction of the tracking axes in the head's own (forward, left, up): `head` takes the
// head's axes to the tracking axes.
[[nodiscard]] math::Vector3 in_head_axes(const math::Vector3& vector, const math::Quaternion& head) noexcept;

// From the head's axes (forward, left, up) to the axes of camera space, which are the
// universe's: `universe_to_camera` has the camera's right, up and backward as rows.
[[nodiscard]] math::Vector3 head_to_camera_space(const math::Vector3& vector,
                                                 const math::Matrix3& universe_to_camera) noexcept;

// A turn of the hand's axes, given in the head's, as columns (the hand's forward, left and up)
// in camera space.
[[nodiscard]] math::Matrix3 hand_to_camera_space(const math::Quaternion& hand_in_head,
                                                 const math::Matrix3& universe_to_camera) noexcept;

// What the arm does while a hand holds on (D27): the hand stays where it holds, so the body
// moves relative to the hold as the hand moved relative to the body, the other way. `moved_m`
// is how far the hand moved over `dt_s`, in the pilot's axes.
[[nodiscard]] math::Vector3 arm_from_hand(const math::Vector3& moved_m, double dt_s) noexcept;

// The thumbstick as a wanted movement in the pilot's axes (forward, left, up).
[[nodiscard]] math::Vector3 stick_to_move(const Controller& controller) noexcept;

} // namespace helios::app

#endif // HELIOS_APPS_HELIOS_VR_HANDS_HPP
