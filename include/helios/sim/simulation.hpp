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
    // Takes the pilot's own axes (forward, left, up: where the eyes look) to those of what the
    // pilot moves about on: the vessel's in a cabin, the ground's (the body-fixed axes of the
    // vessel's domain body) on foot outside.
    math::Quaternion view;
    // Wanted movement along the pilot's axes, each from -1 to 1: walking when standing. Nothing
    // moves a body that floats free.
    math::Vector3 move;
    // Where the hands reach, in the pilot's axes (zero: straight ahead): what is in reach that
    // way, a wall or a loose item, is what a hand takes hold of.
    math::Vector3 hand;
    // How fast the arm moves the body relative to what the hand holds, in m/s, in the pilot's
    // axes (D27): the hand that grabs, or else the one that latched. What it holds is pushed the
    // other way: a wall hardly moves, a loose item does.
    math::Vector3 arm_m_s;
    // The grabbing hand is closed: it takes hold of what is in reach as it closes, and lets go
    // when it opens, the body and what it held keeping their speeds.
    bool grab = false;
    // The latching hand is closing: as it closes it takes hold of what is in reach (moving there
    // from what it held), and it keeps that hold when it opens; only the grabbing hand taking
    // over what it holds frees it. Nothing in reach: it keeps the hold it had.
    bool latch = false;
    bool jump = false;
};

// What the pilot can do where they are, with the one key that does "the obvious thing": each
// frame the simulation says what that would be, and Simulation::interact() does it.
enum class PilotOffer : std::uint8_t {
    None,
    LeaveSeat,
    TakeSeat,

    GoOutside,  // through the airlock within reach, in a suit
    ComeInside, // through the airlock the suited pilot is at
    TakeLadder, // the ladder within reach of the suited pilot
    LetGo,      // of the ladder the suited pilot holds
};

// The suit's safety line (BRIEFING D36): clipped on at the airlock on the way out, in free
// fall, and made fast there to the vessel the pilot came out of. It is slack until the suit is
// as far from that point as its length, and then holds the two together.
struct SuitTether {
    VesselId host;
    math::Vector3 host_point_m; // where it is made fast: host's axes, from the host's origin
    double length_m = 0.0;
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
    bool held = false;            // a hand of the pilot holds it
};

// The player's own body. Aboard a vessel it is either in the seat or moving about the cabin,
// in the vessel's frame: it floats when the vessel is in free fall and has weight when the
// vessel is pushed (by its engines, or by the ground). Outside, the pilot is in a suit, which
// is a small vessel of its own (BRIEFING D31): `vessel` is then the suit. The pilot is "seated"
// in it while it flies, and on foot (not seated) where there is ground near and weight to stand
// with: the legs then hold the suit up and walk it (BRIEFING D32), and `view` and `up` are in
// the ground's axes instead of the vessel's (see PilotInput).
struct Pilot {
    VesselId vessel;
    bool seated = true;
    math::Vector3 position_m;   // of the body's centre from the vessel's origin, vessel axes
    math::Vector3 velocity_m_s; // relative to the vessel, vessel axes
    math::Quaternion view;      // as last asked for
    math::Vector3 up;           // against the weight felt, unit length; zero when weightless
    bool standing = false;      // on its legs: something is under the feet
    bool grabbing = false;      // a hand holds something
    bool latched = false;       // the latching hand holds something
    bool holding = false;       // a hand holds a loose item
    bool in_reach = false;      // something could be taken hold of where the hands reach
    math::Vector3 grip_m;       // where the hand the arm moves holds, vessel axes from the vessel's origin
    math::Vector3 grip_normal;  // of the surface held there, out of it, vessel axes
    PilotOffer offer = PilotOffer::None;
    bool outside = false; // in a suit
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

struct LegsTick;
struct LegPush;
struct LadderTick;

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
    // Takes hold of the ladder within reach (BRIEFING D35): from then on the arm holds the
    // body off the hull, and the pilot climbs it with the walking keys. Only a pilot outside
    // can, and only a ladder within arm's reach of where the suit is.
    [[nodiscard]] core::VoidResult grab_ladder();
    // Lets go of the ladder: on foot below, or floating free.
    [[nodiscard]] core::VoidResult let_go_ladder();
    // Whether the suited pilot is holding a ladder now, and whether one is within reach.
    [[nodiscard]] bool on_ladder() const noexcept { return ladder_latch_.has_value(); }
    [[nodiscard]] bool ladder_in_reach() const;
    // The suit's tether while it is clipped on (BRIEFING D36). It is unclipped coming inside.
    [[nodiscard]] const std::optional<SuitTether>& tether() const noexcept { return tether_; }
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
    // Both hands let go of whatever they hold.
    void open_hands();
    // Keeps the pilot's flags of what the hands hold in step with the cabin.
    void note_hands();
    void update_offer();
    [[nodiscard]] core::VoidResult finish_passage();
    // The vessel whose airlock the suited pilot is at.
    [[nodiscard]] std::optional<VesselId> hatch_within_reach() const;
    // On foot outside (BRIEFING D32). What the legs of the suited pilot do to the suit in one
    // tick of the bubble near the ground; and whether the pilot, standing still as a landed
    // vessel, asks them to move again.
    [[nodiscard]] LegPush suit_legs(const LegsTick& tick);
    [[nodiscard]] bool legs_wanted() const noexcept;
    // The pilot's input and look, as someone on foot at `suit` facing the way the suit does.
    void face_as_suit(const Vessel& suit);

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
    // The legs of the pilot on foot outside, from tick to tick.
    struct Legs {
        bool jump_held = false; // the jump input at the latest tick: a jump starts when it goes down
        bool turning = false;   // the body is being turned to where the pilot looks
    };
    Legs legs_;
    // Where the pilot holds on to a ladder, and on which vessel it hangs (BRIEFING D35): how
    // far up the ladder the hand is and how far across the hull from its line.
    struct LadderLatch {
        VesselId host;
        double along_m = 0.0;
        double side_m = 0.0;
    };
    std::optional<LadderLatch> ladder_latch_;
    // A ladder on a vessel, as its datasheet puts it: in the vessel's axes from its origin.
    struct LadderLine {
        VesselId host;
        math::Vector3 from_m;
        math::Vector3 to_m;
        math::Vector3 out_m; // unit, away from the hull
        double side_m = 0.0;
    };
    // The ladder the suited pilot holds, where its host has it this moment; none when the host
    // is gone from the world.
    [[nodiscard]] std::optional<LadderLine> held_ladder() const;
    // The ladder the suited pilot could take hold of now, within `reach_m` of where the suit would
    // hang on it (zero: an arm's reach), and where the pilot would be on it.
    [[nodiscard]] std::optional<LadderLatch> ladder_near(double reach_m = 0.0) const;
    // One tick of holding on: what the arm does to the suit and to the hull it holds (D35).
    [[nodiscard]] LegPush ladder_hold(const LegsTick& tick);
    std::optional<SuitTether> tether_;
    // Makes the tether a line of the bubble's world between the suit and its host, wherever
    // both are bodies of it this tick (D36); lets go of it if they are already too far apart.
    [[nodiscard]] core::VoidResult hold_tether();
};

} // namespace helios::sim

#endif // HELIOS_SIM_SIMULATION_HPP
