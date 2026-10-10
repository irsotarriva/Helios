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
    Stowed,  // taken aboard another vessel (a suit put away): not in the world
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
    // What an accelerometer aboard reads (universe axes): the acceleration that is not free
    // fall. Thrust over mass in flight, the push of the ground on a landed vessel.
    math::Vector3 proper_acceleration_m_s2;

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

// The pilot's eyes are this far above the centre of the body, along the body's own up.
inline constexpr double k_pilot_eye_height_m = 0.2;

// What the player asks of the pilot for the time being (BRIEFING §12, D27).
struct PilotInput {
    // Takes the pilot's own axes (forward, left, up: where the eyes look) to the vessel's.
    math::Quaternion view;
    // Wanted movement along the pilot's axes, each from -1 to 1: walking when standing, the
    // body relative to the gripping hand when holding on. Nothing moves a body that floats free.
    math::Vector3 move;
    bool grab = false; // hold on to the surface in reach along the line of sight
    bool jump = false;
};

// What the pilot can do where they are, with the one key that does "the obvious thing": each
// frame the simulation says what that would be, and Simulation::interact() does it.
enum class PilotOffer : std::uint8_t {
    None,
    LeaveSeat,
    TakeSeat,
    PickUp,     // the loose item the pilot looks at
    PutDown,    // the item in hand
    GoOutside,  // through the airlock within reach, in a suit
    ComeInside, // through the airlock the suited pilot is at
};

// Something loose in the cabin of the pilot's vessel (BRIEFING D27): a box with a mass.
struct CabinItem {
    std::string name;
    math::Vector3 size_m;
    double mass_kg = 1.0;
    math::Vector3 colour;         // red, green, blue in [0, 1]
    math::Vector3 position_m;     // of its centre from the vessel's origin, vessel axes
    math::Quaternion orientation; // takes its axes to the vessel's
    math::Vector3 velocity_m_s;   // relative to the vessel
    bool held = false;            // in the pilot's hand: it goes where the pilot goes
};

// The player's own body. Aboard a vessel it is either in the seat or moving about the cabin,
// in the vessel's frame: it floats when the vessel is in free fall and has weight when the
// vessel is pushed (by its engines, or by the ground). Outside, the pilot is in a suit, which
// is a small vessel of its own (BRIEFING D31): `vessel` is then the suit, and the pilot is
// "seated" in it.
struct Pilot {
    VesselId vessel;
    bool seated = true;
    math::Vector3 position_m;   // of the body's centre from the vessel's origin, vessel axes
    math::Vector3 velocity_m_s; // relative to the vessel, vessel axes
    math::Quaternion view;      // as last asked for
    math::Vector3 up;           // against the weight felt, unit length; zero when weightless
    bool standing = false;      // on its legs
    bool grabbing = false;
    bool in_reach = false; // a surface could be grabbed where the pilot looks
    math::Vector3 grip_m;  // where the hand holds, vessel axes from the vessel's origin
    PilotOffer offer = PilotOffer::None;
    std::string offer_item;               // the name of the item a PickUp or PutDown is about
    std::optional<std::size_t> held_item; // index into Simulation::cabin_items()
    bool outside = false;                 // in a suit
    // Going through an airlock (suiting up and letting the air out, or the reverse): how long
    // it still takes, and how long it takes in all. Nothing else can be done meanwhile.
    double airlock_left_s = 0.0;
    double airlock_cycle_s = 0.0;
    bool airlock_outwards = false;
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
    // A frame that would need more cabin ticks than this (a long stall) leaves the pilot where
    // they are instead of catching up.
    int max_cabin_ticks_per_frame = 64;
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
    // Flying vessels close to it are in the bubble with it and can touch it and each other
    // (BRIEFING D30). Every other vessel, and all of them at higher warp, are on rails.
    [[nodiscard]] core::VoidResult set_active_vessel(std::optional<VesselId> id);
    [[nodiscard]] std::optional<VesselId> active_vessel() const noexcept { return active_vessel_; }
    [[nodiscard]] bool in_bubble(VesselId id) const noexcept;

    // Puts the pilot in the seat of a vessel (one of whose parts has a cockpit).
    [[nodiscard]] core::VoidResult board(VesselId id);
    // Out of the seat, into the cabin; only where the cockpit says there is a cabin to move in.
    [[nodiscard]] core::VoidResult leave_seat();
    // Back into the seat, from within reach of it.
    [[nodiscard]] core::VoidResult take_seat();
    // Stays in effect until the next one.
    void set_pilot_input(const PilotInput& input) noexcept { pilot_input_ = input; }
    [[nodiscard]] const std::optional<Pilot>& pilot() const noexcept { return pilot_; }
    // Does what Pilot::offer says: leaves or takes the seat, picks an item up, puts it down.
    [[nodiscard]] core::VoidResult interact();
    // Throws the item in hand where the pilot looks. The pilot goes the other way with as much
    // momentum.
    [[nodiscard]] core::VoidResult throw_item();
    // The suit an airlock hands out: a vessel's systems, whose crewed part is what the pilot
    // wears. With a thruster pack among its parts it can fly; without one it only drifts.
    // Until a suit is provided nobody can go outside.
    void provide_suit(vessel::VesselSystems suit);
    // Out through the airlock within reach of a pilot who is out of the seat. The pilot is
    // outside, as the suit, when the airlock has cycled.
    [[nodiscard]] core::VoidResult go_outside();
    // Back in through the airlock the suited pilot is at. The suit is put away at once; the
    // pilot can move about the cabin when the airlock has cycled.
    [[nodiscard]] core::VoidResult come_inside();
    // The loose items of the cabin the pilot is in; empty in a vessel without a cabin.
    [[nodiscard]] std::span<const CabinItem> cabin_items() const noexcept { return cabin_items_; }

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
    // Writes every vessel's "nav/…" signals for the current epoch.
    [[nodiscard]] core::VoidResult report_navigation();
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
    // Whether a vessel is near enough to the bubble's anchor to be a rigid body with it.
    [[nodiscard]] bool within_bubble(std::size_t index, double distance_m) const noexcept;
    // Whether a vessel other than `index` flies within `distance_m` of it.
    [[nodiscard]] bool flown_near(std::size_t index, double distance_m) const noexcept;
    [[nodiscard]] core::VoidResult join_bubble(std::size_t index, const time::Epoch& since);
    // Back on rails from `epoch`, where the vessel's propagator stands.
    [[nodiscard]] core::VoidResult release_from_bubble(std::size_t index, const time::Epoch& epoch);
    [[nodiscard]] core::VoidResult advance_in_bubble(const time::Epoch& instant);
    [[nodiscard]] core::VoidResult bubble_tick();
    [[nodiscard]] core::VoidResult restart_propagator(Vessel& vessel, const dynamics::VesselState& state);
    [[nodiscard]] core::VoidResult hold_attitude(std::size_t index, const dynamics::VesselState& state,
                                                 const vessel::MassProperties& mass, bool touching,
                                                 std::vector<vessel::Separation>& separations);
    [[nodiscard]] core::VoidResult advance_landed(std::size_t index, const time::Epoch& instant);
    void place_landed(Vessel& vessel, const time::Epoch& instant) const;
    [[nodiscard]] core::VoidResult lift_off(std::size_t index);
    [[nodiscard]] std::optional<WarpLimit> pending_domain_change(VesselId id) const;

    // The pilot in the cabin (simulation_pilot.cpp).
    struct Cabin;
    [[nodiscard]] core::VoidResult advance_pilot(const time::Epoch& instant);
    [[nodiscard]] core::VoidResult cabin_tick(double step_s);
    [[nodiscard]] core::VoidResult build_cabin();
    void seat_pilot();
    [[nodiscard]] core::VoidResult pick_up(std::size_t item);
    // Lets go of the item in hand, at `speed_m_s` along the line of sight relative to the pilot.
    [[nodiscard]] core::VoidResult release_item(double speed_m_s);
    void update_offer();
    [[nodiscard]] core::VoidResult finish_passage();
    // The vessel whose airlock the suited pilot is at.
    [[nodiscard]] std::optional<VesselId> hatch_within_reach() const;

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
    std::optional<Pilot> pilot_;
    PilotInput pilot_input_;
    std::unique_ptr<Cabin> cabin_;
    std::vector<CabinItem> cabin_items_;
    // What goes on in the cabin does to the vessel, averaged over the latest frame: the opposite
    // of every push the cabin gave the pilot and the loose items. Vessel axes; the torque is
    // about the vessel's origin.
    struct CabinReaction {
        math::Vector3 force_n;
        math::Vector3 torque_n_m;
    };
    std::optional<CabinReaction> cabin_reaction_;
    // Through an airlock, one way or the other (BRIEFING D31).
    struct Passage {
        bool outwards = true;
        time::Epoch ends;
    };
    std::optional<Passage> passage_;
    std::optional<vessel::VesselSystems> suit_; // as handed out the first time
    std::optional<VesselId> suit_vessel_;       // the suit's place among the vessels, once worn
};

} // namespace helios::sim

#endif // HELIOS_SIM_SIMULATION_HPP
