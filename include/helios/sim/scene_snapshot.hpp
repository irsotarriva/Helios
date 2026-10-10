#ifndef HELIOS_SIM_SCENE_SNAPSHOT_HPP
#define HELIOS_SIM_SCENE_SNAPSHOT_HPP

#include "helios/bodies/body_catalog.hpp"
#include "helios/core/error.hpp"
#include "helios/dynamics/events.hpp"
#include "helios/math/matrix3.hpp"
#include "helios/math/quaternion.hpp"
#include "helios/math/vector3.hpp"
#include "helios/orbital/conic.hpp"
#include "helios/sim/simulation.hpp"
#include "helios/time/epoch.hpp"
#include "helios/vessel/part_datasheet.hpp"

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace helios::sim {

// What the map view looks at. Everything in a snapshot is positioned relative to the focus.
struct Focus {
    enum class Kind : std::uint8_t { Body, Vessel };
    Kind kind = Kind::Body;
    std::uint32_t index = 0; // a BodyId or VesselId index

    [[nodiscard]] static Focus body(bodies::BodyId id) noexcept { return {Kind::Body, id.index}; }
    [[nodiscard]] static Focus vessel(VesselId id) noexcept { return {Kind::Vessel, id.index}; }
    [[nodiscard]] friend constexpr bool operator==(const Focus&, const Focus&) noexcept = default;
};

struct BodyView {
    bodies::BodyId id;
    std::string name;
    math::Vector3 position_m; // relative to the focus, universe axes
    double radius_m = 0.0;
    std::optional<bodies::BodyId> domain_parent; // none for the root of the domain hierarchy (the star)
    // Columns are the body-fixed axes in universe axes (identity if the body has no rotation model).
    math::Matrix3 body_to_universe;
};

// One signal of a vessel's control bus (BRIEFING §8.3), as a control or a gauge shows it.
struct SignalView {
    std::string name;
    std::string unit;
    bool command = false; // something to set; otherwise telemetry to read
    bool toggle = false;  // a switch rather than a lever
    double value = 0.0;
    double minimum = 0.0;
    double maximum = 0.0;
};

// One part of a vessel, for whatever draws it.
struct PartView {
    std::string name;
    math::Vector3 position_m;  // of the part's origin from the vessel's centre of mass, vessel axes
    math::Matrix3 orientation; // takes the part's axes to the vessel's
    std::shared_ptr<const vessel::PartDatasheet> datasheet; // its shape and, if crewed, its cockpit
};

struct VesselView {
    VesselId id;
    std::string name;
    VesselStatus status = VesselStatus::Flying;
    math::Vector3 position_m;
    math::Vector3 velocity_in_domain_m_s;
    double surface_speed_m_s = 0.0;  // relative to the turning surface of the domain body
    double vertical_speed_m_s = 0.0; // away from its centre
    bodies::BodyId domain;
    std::string domain_name;
    double domain_radius_m = 0.0;                     // the domain body's surface
    std::optional<orbital::ConicGeometry> osculating; // about the domain body; none if degenerate
    std::size_t pending_burns = 0;
    bool thrusting = false;
    bool in_bubble = false;       // simulated as a rigid body (near the active vessel, at low warp)
    math::Quaternion orientation; // takes the vessel's axes (+x to the nose) to the universe's
    double turn_rate_rad_s = 0.0;
    std::vector<SignalView> signals; // empty for a vessel without systems
    std::vector<PartView> parts;     // empty for a vessel without systems, or one destroyed
};

// The player's body aboard a vessel (BRIEFING D27). Positions are in the vessel's axes from
// its centre of mass, like those of its parts.
struct PilotView {
    VesselId vessel;
    bool seated = true;
    math::Vector3 position_m; // of the body's centre; the eyes are k_pilot_eye_height_m above it
    math::Vector3 velocity_m_s;
    math::Vector3 up; // against the weight felt, unit length; zero when weightless
    bool standing = false;
    bool grabbing = false;
    bool in_reach = false; // a surface could be grabbed where the pilot looks
    math::Vector3 grip_m;
    PilotOffer offer = PilotOffer::None; // what the interact key would do now
    std::string offer_item;              // the item a PickUp or PutDown is about
    bool holding = false;                // an item is in hand
    bool outside = false;                // in a suit: `vessel` is the suit
    double airlock_left_s = 0.0;         // going through an airlock: what is left of the wait ...
    double airlock_cycle_s = 0.0;        // ... and all of it
    bool airlock_outwards = false;       // on the way out (suiting up), not in
};

// A loose item in the cabin of the pilot's vessel; placed like the vessel's parts.
struct ItemView {
    std::string name;
    math::Vector3 position_m;
    math::Quaternion orientation; // takes the item's axes to the vessel's
    math::Vector3 size_m;
    math::Vector3 colour;
    bool held = false; // in the pilot's hand: drawn there, wherever the pilot looks
};

enum class LineKind : std::uint8_t {
    BodyOrbit,
    VesselTrajectory, // the path a coasting vessel will follow, planned burns included
    // While a vessel is under thrust its trajectory is drawn as two lines instead:
    VesselOrbit,    // the orbit it would be left on if the thrust stopped now
    VesselBurnPath, // the path it will follow if the thrust goes on as planned
};

struct LineView {
    LineKind kind = LineKind::BodyOrbit;
    std::uint32_t owner = 0;             // BodyId or VesselId index
    bodies::BodyId frame_body;           // the body the line is drawn relative to (the orbit's centre)
    std::vector<math::Vector3> points_m; // relative to the focus
    bool closed = false;
};

struct EventView {
    VesselId vessel;
    std::optional<dynamics::EventKind> kind; // none for a scheduled burn
    bodies::BodyId body;
    double time_to_event_s = 0.0;
};

// Everything the renderer needs for one frame, immutable once built.
//
// Floating origin (BRIEFING §9): positions are doubles relative to the focus, which is the
// natural origin for the camera. The renderer subtracts its camera offset (also a double
// relative to the focus) and only then converts to float, so the GPU never sees a coordinate
// larger than the view requires, whatever the distance from the Sun.
struct SceneSnapshot {
    time::Epoch epoch;
    double warp_factor = 1.0;
    double requested_warp_factor = 1.0;
    double max_warp_factor = 1.0; // the highest factor that can be requested
    bool paused = false;
    Focus focus;
    std::string focus_name;
    double focus_radius_m = 0.0; // the focus body's radius (0 for a vessel)
    std::vector<BodyView> bodies;
    std::vector<VesselView> vessels;
    std::vector<LineView> lines;
    std::optional<EventView> next_event;
    std::optional<PilotView> pilot; // none until the pilot has boarded a vessel
    std::vector<ItemView> items;    // of the cabin the pilot is in
};

struct SnapshotOptions {
    std::size_t body_orbit_points = 256;
};

[[nodiscard]] core::Result<SceneSnapshot> build_snapshot(const Simulation& simulation, const Focus& focus,
                                                         const SnapshotOptions& options = {});

// Hands the latest snapshot from the simulation thread to the render thread. The renderer only
// ever reads a complete, immutable snapshot; the simulation never waits for the renderer.
class SnapshotExchange {
public:
    void publish(std::shared_ptr<const SceneSnapshot> snapshot);
    [[nodiscard]] std::shared_ptr<const SceneSnapshot> latest() const;

private:
    // Rationale: a mutex rather than std::atomic<std::shared_ptr>, which Apple's libc++ lacks.
    mutable std::mutex mutex_;
    std::shared_ptr<const SceneSnapshot> latest_;
};

} // namespace helios::sim

#endif // HELIOS_SIM_SCENE_SNAPSHOT_HPP
