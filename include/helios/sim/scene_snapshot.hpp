#ifndef HELIOS_SIM_SCENE_SNAPSHOT_HPP
#define HELIOS_SIM_SCENE_SNAPSHOT_HPP

#include "helios/bodies/body_catalog.hpp"
#include "helios/core/error.hpp"
#include "helios/dynamics/events.hpp"
#include "helios/math/matrix3.hpp"
#include "helios/math/vector3.hpp"
#include "helios/orbital/conic.hpp"
#include "helios/sim/simulation.hpp"
#include "helios/time/epoch.hpp"

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

struct VesselView {
    VesselId id;
    std::string name;
    VesselStatus status = VesselStatus::Flying;
    math::Vector3 position_m;
    math::Vector3 velocity_in_domain_m_s;
    bodies::BodyId domain;
    std::string domain_name;
    double domain_radius_m = 0.0;                     // the domain body's surface
    std::optional<orbital::ConicGeometry> osculating; // about the domain body; none if degenerate
    std::size_t pending_burns = 0;
};

enum class LineKind : std::uint8_t {
    BodyOrbit,
    VesselTrajectory,
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
    bool paused = false;
    Focus focus;
    std::string focus_name;
    double focus_radius_m = 0.0; // the focus body's radius (0 for a vessel)
    std::vector<BodyView> bodies;
    std::vector<VesselView> vessels;
    std::vector<LineView> lines;
    std::optional<EventView> next_event;
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
