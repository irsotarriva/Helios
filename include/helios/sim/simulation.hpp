#ifndef HELIOS_SIM_SIMULATION_HPP
#define HELIOS_SIM_SIMULATION_HPP

#include "helios/bodies/body_catalog.hpp"
#include "helios/core/error.hpp"
#include "helios/dynamics/encke_propagator.hpp"
#include "helios/dynamics/events.hpp"
#include "helios/dynamics/gravity_model.hpp"
#include "helios/dynamics/trajectory.hpp"
#include "helios/frames/frame_tree.hpp"
#include "helios/sim/time_warp.hpp"
#include "helios/time/epoch.hpp"

#include <compare>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace helios::sim {

struct VesselId {
    std::uint32_t index = 0;

    [[nodiscard]] friend constexpr auto operator<=>(const VesselId&, const VesselId&) noexcept = default;
};

enum class VesselStatus : std::uint8_t {
    Flying,
    Crashed, // reached its domain body's surface; frozen until Phase 3 adds landing
};

struct Vessel {
    std::string name;
    VesselStatus status = VesselStatus::Flying;
    dynamics::EnckePropagator propagator;
    dynamics::VesselState state; // at the simulation's current epoch

    // Caches, rebuilt by the simulation when stale. Events change only with the trajectory (a
    // burn, a domain change) or when the first one passes; the display prediction is also
    // extended as time moves along it.
    std::vector<dynamics::PredictedEvent> events;        // from `state`, in time order
    std::vector<dynamics::TrajectorySegment> prediction; // the future path, for the map view
    time::Epoch prediction_start;
    double prediction_horizon_s = 0.0;
    bool events_stale = true;
    bool prediction_stale = true;
};

struct SimulationOptions {
    dynamics::PropagatorOptions propagator;
    dynamics::GravityOptions gravity;
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

    // The state must be in the frame of `initial.domain` at the current epoch.
    [[nodiscard]] core::Result<VesselId>
    add_vessel(std::string name, const orbital::StateVector& state_in_domain, bodies::BodyId domain);

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

    std::unique_ptr<frames::FrameTree> tree_;
    std::unique_ptr<bodies::BodyCatalog> catalog_;
    std::unique_ptr<dynamics::GravityModel> gravity_;
    SimulationOptions options_;
    time::Epoch now_;
    TimeWarp warp_;
    double effective_warp_ = 1.0;
    std::vector<Vessel> vessels_;
};

} // namespace helios::sim

#endif // HELIOS_SIM_SIMULATION_HPP
