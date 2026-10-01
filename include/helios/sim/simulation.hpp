#ifndef HELIOS_SIM_SIMULATION_HPP
#define HELIOS_SIM_SIMULATION_HPP

#include "helios/bodies/body_catalog.hpp"
#include "helios/core/error.hpp"
#include "helios/dynamics/encke_propagator.hpp"
#include "helios/dynamics/events.hpp"
#include "helios/dynamics/gravity_model.hpp"
#include "helios/dynamics/trajectory.hpp"
#include "helios/frames/frame_tree.hpp"
#include "helios/math/quaternion.hpp"
#include "helios/math/vector3.hpp"
#include "helios/sim/time_warp.hpp"
#include "helios/time/epoch.hpp"
#include "helios/vessel/vessel_systems.hpp"

#include <compare>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace helios::sim {

struct VesselId {
    std::uint32_t index = 0;

    [[nodiscard]] friend constexpr auto operator<=>(const VesselId&, const VesselId&) noexcept = default;
};

enum class VesselStatus : std::uint8_t {
    Flying,
    Landed,  // at rest on its domain body and turning with it
    Crashed, // destroyed on its domain body's surface
};

// How a vessel is turned. Its own axes have +x towards the nose.
struct Attitude {
    math::Quaternion orientation;         // takes the vessel's axes to the universe's
    math::Vector3 angular_velocity_rad_s; // universe axes
};

// Where a landed vessel stands, in the body-fixed (rotating) frame of its domain body.
struct LandedPlace {
    math::Vector3 position_m;     // of its centre of mass
    math::Quaternion orientation; // takes the vessel's axes to the body-fixed ones
};

struct Vessel {
    std::string name;
    VesselStatus status = VesselStatus::Flying;
    dynamics::EnckePropagator propagator;
    dynamics::VesselState state; // at the simulation's current epoch
    // Parts, stores and the control bus (BRIEFING D22); none for a bare point on a trajectory.
    std::optional<vessel::VesselSystems> systems;
    // For a vessel with systems. In the physics bubble this is integrated; on rails it is the
    // commanded pointing, held exactly (BRIEFING D22, D25).
    Attitude attitude;
    std::optional<LandedPlace> landed; // set while the status is Landed

    // Caches, rebuilt by the simulation when stale. Events change with the trajectory (a burn,
    // a domain change), when the first one passes, and as the search horizon moves on with
    // time; the display prediction is also extended as time moves along it.
    std::vector<dynamics::PredictedEvent> events;        // from `state`, in time order
    time::Epoch events_epoch;                            // when they were predicted
    std::vector<dynamics::TrajectorySegment> prediction; // the future path, for the map view
    time::Epoch prediction_start;
    double prediction_horizon_s = 0.0;
    bool events_stale = true;
    bool prediction_stale = true;
};

struct SimulationOptions {
    // Rationale for 1e-6: it puts low orbits on exact conics (the lunar and solar tides on a
    // low Earth orbit are ~2e-7 of Earth's gravity), which is what makes them affordable at
    // any warp, while geostationary, lunar and interplanetary trajectories (≥ 1e-5) keep their
    // perturbations.
    dynamics::PropagatorOptions propagator{.analytic_perturbation_ratio = 1e-6};
    dynamics::GravityOptions gravity;
    // The highest warp factor the player can request (BRIEFING §7), rounded down to a level of
    // k_warp_levels.
    double max_warp_factor = 1e9;
    // The physics bubble (BRIEFING §5.3, D25) runs up to this warp, on a fixed step.
    // Rationale: 1/128 s is an exact binary fraction, so tick epochs carry no rounding.
    double max_physics_warp = 4.0;
    double physics_step_s = 1.0 / 128.0;
    double event_horizon_s = 365.25 * 86'400.0;
    // Display prediction: one orbital period for bound orbits, this long otherwise.
    double unbound_prediction_horizon_s = 60.0 * 86'400.0;
    double max_prediction_horizon_s = 2.0 * 365.25 * 86'400.0;
    std::size_t prediction_samples = 2048;
};

// Why the warp is currently limited: the earliest upcoming event that the player should not
// skip over (a burn, a sphere-of-influence change, an impact).
struct WarpLimit {
    VesselId vessel;
    time::Epoch epoch;
    std::optional<dynamics::EventKind> kind; // none for a scheduled impulse
    bodies::BodyId body;
};

// The headless universe: bodies on rails, vessels propagated by Encke, the universe clock and
// the time-warp controller. It owns everything the propagators reference, on the heap, so a
// Simulation can be moved freely. Single-threaded; the client runs it on one thread and hands
// SceneSnapshots to the renderer (scene_snapshot.hpp).
class Simulation {
public:
    [[nodiscard]] static core::Result<Simulation> make(std::unique_ptr<frames::FrameTree> tree,
                                                       std::unique_ptr<bodies::BodyCatalog> catalog,
                                                       const time::Epoch& start, SimulationOptions options);

    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;
    Simulation(Simulation&&) noexcept;
    Simulation& operator=(Simulation&&) noexcept;
    ~Simulation();

    // The state must be in the frame of `initial.domain` at the current epoch.
    [[nodiscard]] core::Result<VesselId>
    add_vessel(std::string name, const orbital::StateVector& state_in_domain, bodies::BodyId domain);

    // A vessel made of parts. Its systems must not be ahead of the current epoch, nor under
    // thrust since before it.
    [[nodiscard]] core::Result<VesselId> add_vessel(std::string name,
                                                    const orbital::StateVector& state_in_domain,
                                                    bodies::BodyId domain, vessel::VesselSystems systems);

    // A vessel made of parts, standing on `body` nose up at a planetocentric latitude and
    // longitude, and told to keep pointing up.
    [[nodiscard]] core::Result<VesselId> add_landed_vessel(std::string name, bodies::BodyId body,
                                                           double latitude_rad, double longitude_rad,
                                                           vessel::VesselSystems systems);

    // The vessel the player flies. While the warp is at most max_physics_warp it is in the
    // physics bubble: a rigid body that turns under torques, touches the ground and can land.
    // Every other vessel, and this one at higher warp, is on rails.
    [[nodiscard]] core::VoidResult set_active_vessel(std::optional<VesselId> id);
    [[nodiscard]] std::optional<VesselId> active_vessel() const noexcept { return active_vessel_; }
    [[nodiscard]] bool in_bubble(VesselId id) const noexcept;

    // Publishes a command on the vessel's control bus now (BRIEFING §8.3), or releases the
    // source's hold on the signal. Engines that start or stop as a result change the thrust on
    // the trajectory from this instant; parts that separate become vessels of their own.
    [[nodiscard]] core::VoidResult command(VesselId id, std::string_view signal, vessel::ControlSource source,
                                           double value);
    [[nodiscard]] core::VoidResult release_command(VesselId id, std::string_view signal,
                                                   vessel::ControlSource source);
    // A command to be published when its epoch is reached: a planned burn is two of these.
    [[nodiscard]] core::VoidResult schedule_command(VesselId id, vessel::TimedCommand command);

    [[nodiscard]] core::VoidResult schedule_impulse(VesselId id, const dynamics::Impulse& impulse);

    // An impulse given in the vessel's prograde / normal / radial-out frame *at the burn epoch*
    // (orbital/conic.hpp: ManeuverBasis), evaluated on the vessel's exact future state.
    [[nodiscard]] core::VoidResult schedule_maneuver(VesselId id, const time::Epoch& epoch,
                                                     double prograde_m_s, double normal_m_s,
                                                     double radial_out_m_s);

    // Advances the clock by one frame of wall time at the (event-limited) warp. Reaching a
    // scheduled impulse or an impact drops the requested warp to real time.
    [[nodiscard]] core::VoidResult advance(double wall_dt_s);

    // Moves the clock to `instant` (≥ now) regardless of warp. Used by tests and tools.
    [[nodiscard]] core::VoidResult advance_to(const time::Epoch& instant);

    [[nodiscard]] const time::Epoch& now() const noexcept { return now_; }
    [[nodiscard]] TimeWarp& time_warp() noexcept { return warp_; }
    [[nodiscard]] const TimeWarp& time_warp() const noexcept { return warp_; }
    [[nodiscard]] double effective_warp() const noexcept { return effective_warp_; }
    [[nodiscard]] std::optional<WarpLimit> warp_limit() const;

    [[nodiscard]] const frames::FrameTree& tree() const noexcept { return *tree_; }
    [[nodiscard]] const bodies::BodyCatalog& catalog() const noexcept { return *catalog_; }
    [[nodiscard]] const dynamics::GravityModel& gravity() const noexcept { return *gravity_; }
    [[nodiscard]] std::span<const Vessel> vessels() const noexcept { return vessels_; }
    [[nodiscard]] core::Result<std::reference_wrapper<const Vessel>> vessel(VesselId id) const noexcept;

private:
    Simulation(std::unique_ptr<frames::FrameTree> tree, std::unique_ptr<bodies::BodyCatalog> catalog,
               std::unique_ptr<dynamics::GravityModel> gravity, const time::Epoch& start,
               SimulationOptions options) noexcept;

    void refresh_caches(Vessel& vessel);
    [[nodiscard]] core::Result<std::reference_wrapper<vessel::VesselSystems>> systems_of(VesselId id);
    [[nodiscard]] core::VoidResult change_command(VesselId id, std::string_view signal,
                                                  vessel::ControlSource source, std::optional<double> value);
    [[nodiscard]] core::VoidResult systems_changed(Vessel& vessel, const time::Epoch& instant);
    [[nodiscard]] core::VoidResult separate(std::size_t parent, vessel::Separation separation);
    [[nodiscard]] core::VoidResult separate_landed(std::size_t parent, vessel::Separation separation);

    // The physics bubble (simulation_bubble.cpp).
    struct Bubble;
    [[nodiscard]] bool bubble_wanted(std::size_t index) const noexcept;
    [[nodiscard]] core::VoidResult update_bubble_membership();
    [[nodiscard]] core::VoidResult enter_bubble(std::size_t index);
    [[nodiscard]] core::VoidResult leave_bubble();
    [[nodiscard]] core::VoidResult advance_in_bubble(const time::Epoch& instant);
    [[nodiscard]] core::VoidResult bubble_tick();
    [[nodiscard]] core::VoidResult hold_attitude(std::size_t index, const dynamics::VesselState& state,
                                                 const vessel::MassProperties& mass,
                                                 std::vector<vessel::Separation>& separations);
    [[nodiscard]] core::VoidResult advance_landed(std::size_t index, const time::Epoch& instant);
    void place_landed(Vessel& vessel, const time::Epoch& instant) const;
    [[nodiscard]] core::VoidResult lift_off(std::size_t index);
    [[nodiscard]] std::optional<WarpLimit> pending_domain_change(VesselId id) const;

    std::unique_ptr<frames::FrameTree> tree_;
    std::unique_ptr<bodies::BodyCatalog> catalog_;
    std::unique_ptr<dynamics::GravityModel> gravity_;
    SimulationOptions options_;
    time::Epoch now_;
    TimeWarp warp_;
    double effective_warp_ = 1.0;
    std::vector<Vessel> vessels_;
    std::optional<VesselId> active_vessel_;
    std::unique_ptr<Bubble> bubble_;
};

} // namespace helios::sim

#endif // HELIOS_SIM_SIMULATION_HPP
