#ifndef HELIOS_RENDER_FLIGHT_GEOMETRY_HPP
#define HELIOS_RENDER_FLIGHT_GEOMETRY_HPP

#include "helios/math/matrix3.hpp"
#include "helios/math/quaternion.hpp"
#include "helios/math/vector3.hpp"
#include "helios/render/camera.hpp"
#include "helios/render/mesh.hpp"
#include "helios/render/scene_geometry.hpp"
#include "helios/sim/scene_snapshot.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

// The flight view (BRIEFING §12, D26): the vessel being flown, seen from its seat or from
// outside, with the ground below it. Like the map view's geometry it is GPU-free: a snapshot
// goes in, camera-space floats come out.
namespace helios::render {

// Where the pilot's head is turned, relative to the seat (BRIEFING §12.1 rule 2). Mouse-look
// produces one of these; a headset will produce the same thing.
struct HeadPose {
    static constexpr double k_max_yaw_rad = 2.97;   // 170°
    static constexpr double k_max_pitch_rad = 1.48; // 85°

    double yaw_rad = 0.0;   // to the left
    double pitch_rad = 0.0; // up

    void turn(double yaw_delta_rad, double pitch_delta_rad) noexcept;
};

// A head or an eye as a headset tracks it, relative to where the head belongs: the eye point of
// a seat, or the eyes of a pilot moving about. `position_m` is in that frame's axes (forward,
// left, up); `orientation` takes the tracked thing's own axes (forward, left, up) to them.
struct TrackedPose {
    math::Vector3 position_m;
    math::Quaternion orientation;
};

// A camera: where it is and how it is turned.
struct ViewPoint {
    math::Vector3 position_from_focus_m; // universe axes
    math::Matrix3 universe_to_camera;    // rows: the camera's right, up and backward axes
};

// The first part of the vessel that has a seat.
[[nodiscard]] std::optional<std::size_t> seat_part(const sim::VesselView& vessel) noexcept;

// From the seat of the vessel's cockpit; none for a vessel without one.
[[nodiscard]] std::optional<ViewPoint> seat_view_point(const sim::VesselView& vessel, const HeadPose& head);

// From the eyes of a pilot moving about the cabin, or on foot outside. `view` takes the eyes'
// axes (forward, left, up) to the vessel's, or to the ground's where the pilot has them
// (PilotView::ground).
[[nodiscard]] ViewPoint pilot_view_point(const sim::VesselView& vessel, const sim::PilotView& pilot,
                                         const math::Quaternion& view);

// The same two with a headset on: the head (or one eye of it) where it is tracked, about the
// seat's eye point or about where the pilot's body faces.
[[nodiscard]] std::optional<ViewPoint> seat_view_point(const sim::VesselView& vessel,
                                                       const TrackedPose& tracked);
[[nodiscard]] ViewPoint pilot_view_point(const sim::VesselView& vessel, const sim::PilotView& pilot,
                                         const math::Quaternion& view, const TrackedPose& tracked);

// Takes the seat's axes (forward, left, up) to the vessel's; none for a vessel without a seat.
[[nodiscard]] std::optional<math::Matrix3> seat_to_vessel(const sim::VesselView& vessel);

// From outside, looking at the vessel. The camera's yaw and pitch are taken about the local
// vertical (away from the body the vessel is near), so the horizon is level.
[[nodiscard]] ViewPoint chase_view_point(const sim::SceneSnapshot& snapshot, const sim::VesselView& vessel,
                                         const OrbitCamera& camera);

enum class SolidShape : std::uint8_t {
    Box,      // make_box()
    Cylinder, // make_cylinder()
};

struct SolidInstance {
    SolidShape shape = SolidShape::Box;
    Matrix4f model{}; // the unit shape → camera space
    std::uint32_t abgr = 0;
    float cabin_light = 0.0F; // how much of its light is the cabin's own rather than the star's
};

// The ground near the camera: a cap of the body's sphere that reaches the horizon, finer
// towards the point below the camera. Its vertices carry coordinates on the body's own
// latitude and longitude grid, in cells, so a pattern drawn from them turns with the body.
struct TerrainPatch {
    Mesh mesh; // camera space
    std::uint32_t abgr = 0;
    double cell_m = 0.0; // the side of one grid cell on the ground
};

// One control of the cockpit as the pilot meets it, or the guard over one (BRIEFING D43): where
// a hand (or the mouse) takes it, and which way it moves. A control under a closed guard has
// none: the guard is in the way.
struct ControlHandle {
    std::size_t control = 0; // its index in the cockpit's controls
    bool guard = false;      // the cover over it, not the control
    Float3 centre;           // of what is taken hold of, camera space
    float radius_m = 0.0F;   // within which a pointer is on it
    // Universe axes. What a drag moves along: an axis's whole travel; a switch's way from one
    // position to the next; an encoder's one step; a stick's full deflection to the right (and
    // `travel_up`, along its up). None for a momentary control or a guard.
    Float3 travel;
    Float3 travel_up;
    std::string label;
    bool live = false; // the vessel has every signal it moves
};

struct FlightGeometry {
    FrameGeometry scene; // the bodies and the labels; no orbit lines
    std::vector<SolidInstance> solids;
    std::optional<TerrainPatch> terrain;
    std::vector<ControlHandle> controls; // empty when seen from outside
    std::vector<LineVertex> text;        // the panels' lettering: a triangle list, unlit
    Float3 sun_direction;                // unit vector towards the star, universe axes
};

// `from_seat`: seen from inside. The interior of the seat's part is drawn instead of its
// outside, with its instruments, and the hand of a pilot who holds on to something.
[[nodiscard]] FlightGeometry build_flight_geometry(const sim::SceneSnapshot& snapshot, std::size_t vessel,
                                                   const ViewPoint& view, bool from_seat);

// The direction, in universe axes, of what is seen at a pixel (from the top left).
[[nodiscard]] Float3 ray_through_pixel(float x_px, float y_px, float width_px, float height_px,
                                       double vertical_field_of_view_rad,
                                       const math::Matrix3& universe_to_camera) noexcept;

// A control (or guard) a pointer or a hand is on, and where on it: `side` is how far along its
// travel from the centre, from -1 to 1 (0 when it has no travel). A click on the far side of a
// switch moves it to its next position, on the near side to the one before.
struct ControlPick {
    std::size_t handle = 0; // its index in `controls`
    double side = 0.0;
};

// The nearest live control or guard along a ray from the camera.
[[nodiscard]] std::optional<ControlPick> pick_control(std::span<const ControlHandle> controls,
                                                      const Float3& ray_direction) noexcept;

// How far along `travel` (as a fraction of it) a pointer moving by so many pixels drags what is
// held at `centre`: it stays under the pointer.
[[nodiscard]] double travel_fraction(const Float3& centre, const Float3& travel,
                                     const Matrix4f& view_projection, float width_px, float height_px,
                                     float dx_px, float dy_px) noexcept;

// A hand of a headset's controller (BRIEFING D39), in camera space.
struct ControllerHand {
    math::Vector3 position_m;    // of the fingertips: what touches
    math::Matrix3 hand_to_space; // columns: the hand's forward, left and up
    bool closed = false;         // holding something
};

// The nearest live control or guard a hand at `point_m` (camera space) touches: within its
// radius and `reach_m` more.
[[nodiscard]] std::optional<ControlPick> touch_control(std::span<const ControlHandle> controls,
                                                       const math::Vector3& point_m, double reach_m) noexcept;

// How far along `travel` (as a fraction of it) a hand moving by `moved_m` (camera space) drags
// what it holds: only the part along the travel counts.
[[nodiscard]] double travel_fraction(const Float3& travel, const math::Vector3& moved_m) noexcept;

// Draws the hands of a headset's controllers as blocks, in the seat's light.
void add_controller_hands(FlightGeometry& geometry, std::span<const ControllerHand> hands);

} // namespace helios::render

#endif // HELIOS_RENDER_FLIGHT_GEOMETRY_HPP
