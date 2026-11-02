#ifndef HELIOS_SOURCE_SIM_BUBBLE_HPP
#define HELIOS_SOURCE_SIM_BUBBLE_HPP

// Internal to the sim library: what simulation.cpp and simulation_bubble.cpp share.

#include "helios/core/error.hpp"
#include "helios/dynamics/encke_propagator.hpp"
#include "helios/math/vector3.hpp"
#include "helios/orbital/kepler.hpp"
#include "helios/physics/world.hpp"
#include "helios/sim/simulation.hpp"
#include "helios/time/epoch.hpp"
#include "helios/vessel/vessel_systems.hpp"

#include <algorithm>
#include <cstddef>
#include <optional>
#include <vector>

namespace helios::sim {

// A vessel of the physics bubble: one rigid body. A landed vessel is a body too, a fixed one,
// put each tick where its place on the ground is; it stays pinned there (LandedPlace).
struct BubbleMember {
    std::size_t vessel = 0; // index of the vessel
    // Where the vessel's propagator stood when it came in: it takes part in the ticks that
    // start no earlier.
    time::Epoch since;
    std::optional<physics::BodyId> body;
    // What the body was built from; it is rebuilt when the vessel no longer matches.
    math::Vector3 body_centre_of_mass_m; // vessel axes
    std::size_t body_part_count = 0;
    bool body_fixed = false; // built for the vessel standing on the ground
    double impact_tolerance_m_s = 0.0;
    // How far the centre of mass has moved in the vessel (parts left it) and the propagator has
    // not been told yet: universe axes, given at the start of the next tick.
    math::Vector3 pending_shift_m;
    bool touching = false; // during the latest tick
    bool pushed = false;   // something other than gravity acted since the caches were refreshed
    int ticks_at_rest = 0;
};

// The physics bubble (BRIEFING §5.3, D25, D30): the vessels that are simulated as rigid bodies,
// in one world about the vessel the player flies.
struct Simulation::Bubble {
    physics::World world;
    std::size_t anchor = 0; // index of the active vessel: the frame of the world goes with it
    // Where the members' propagators and attitudes stand: the start of the next physics tick,
    // at most one tick behind the simulation clock.
    time::Epoch epoch;
    std::vector<BubbleMember> members;
    std::optional<physics::BodyId> ground;
    // The suit's tether as a line of the world (D36), and the two bodies it was made fast to:
    // when either is rebuilt, the line went with it.
    struct Tether {
        physics::TetherId id;
        physics::BodyId suit;
        physics::BodyId host;
    };
    std::optional<Tether> tether;

    [[nodiscard]] BubbleMember* find(std::size_t vessel) noexcept {
        const auto found = std::ranges::find(members, vessel, &BubbleMember::vessel);
        return found != members.end() ? &*found : nullptr;
    }
};

// The ladder the suited pilot is holding this tick (BRIEFING D35): where it stands on the hull
// and the body of the vessel it hangs on, in the world's axes as everything else here.
struct LadderTick {
    physics::BodyId host;          // the vessel it hangs on
    physics::BodyState host_state; // that body now
    math::Vector3 from_m;          // the foot of the ladder
    math::Vector3 to_m;            // the top, at the hatch
    math::Vector3 out_m;           // unit, away from the hull
    double side_m = 0.0;           // how far across the hull the hand may go
};

// What the legs of the suited pilot have to work with in one tick of the bubble (BRIEFING D32).
// Positions and vectors are in the world's axes, velocities relative to the ground.
struct LegsTick {
    const physics::World* world = nullptr;
    physics::BodyId body; // the suit's
    physics::BodyState before;
    double mass_kg = 0.0;
    math::Matrix3 inertia_kg_m2;         // the suit's axes, about its centre of mass
    math::Vector3 apparent_gravity_m_s2; // what a body at rest on the ground is held up against
    math::Matrix3 ground_to_world;       // the body-fixed axes of the domain body, to the world's
    double standing_height_m = 0.0;      // of the centre of mass over the suit's lowest point
    double step_s = 0.0;
    std::optional<LadderTick> ladder; // what the pilot holds on to, while latched (D35)
};

// What the legs do: a push on the suit through its centre of mass and a torque, both from what
// is under the feet, which is pushed the other way; and what the arm does while the pilot holds
// a ladder: then the hull is pushed the other way, at the hand (D35).
struct LegPush {
    math::Vector3 force_n;
    math::Vector3 torque_n_m;
    std::optional<physics::BodyId> footing;   // what the feet are on
    math::Vector3 foot_m;                     // where
    std::optional<physics::BodyId> grip_body; // what the hand holds, if anything
    math::Vector3 grip_m;                     // where it holds it
    bool standing = false;
    bool busy = false; // walking, turning, jumping, climbing or getting up: not at rest
};

// The inside of the vessel the pilot moves about in (BRIEFING D27): its walls, at rest in the
// vessel's axes, and the pilot's body.
struct Simulation::Cabin {
    physics::World world;
    std::optional<physics::BodyId> body;               // the pilot's, while out of the seat
    std::vector<std::optional<physics::BodyId>> items; // per cabin item
    std::size_t part_count = 0;                        // of the vessel the walls were built from
    time::Epoch epoch;                                 // where the cabin's physics stands
    math::Vector3 spin_rad_s;                          // the vessel's at the latest tick, vessel axes
    bool has_spin = false;
    double heavy_s = 0.0;    // for how long the weight felt has been enough to stand against
    bool grab_held = false;  // the grab input at the latest tick: a grip starts when it goes down
    bool latch_held = false; // the same for the latch input
    bool jump_held = false;
    // A hand's hold (D27): a point of a wall (the vessel) or of a loose item, and where the arm
    // wants the body from it.
    struct Hand {
        std::optional<std::size_t> item; // index into the items; none: a wall
        math::Vector3 point_m;           // the item's axes from its centre, or the vessel's from its origin
        math::Vector3 normal;            // of the surface there, out of it, in the same axes
        math::Vector3 offset_m;          // the body's centre from the point, vessel axes
    };
    std::optional<Hand> grabbing; // the hand held closed (PilotInput::grab)
    std::optional<Hand> latched;  // the hand that keeps its hold (Simulation::latch)
    // What the cabin did to the bodies in it over the ticks of this frame, for the vessel to
    // feel the opposite of: the sums of force × time and of torque × time about the origin.
    math::Vector3 impulse_n_s;
    math::Vector3 turning_impulse_n_m_s;
    double ticked_s = 0.0;
};

namespace detail {

[[nodiscard]] dynamics::ThrustDirection to_thrust_direction(const vessel::Pointing& pointing) noexcept;

// Hands the propagator the thrust the vessel's systems will produce from `from` on: the instant
// the propagator has been advanced to. Changes before it are taken to be applied already,
// unless `include_current` says the propagator knows nothing of the thrust in effect (a vessel
// coming out of the physics bubble).
[[nodiscard]] core::VoidResult sync_propulsion(Vessel& vessel, const time::Epoch& from, bool include_current);

// The attitude a vessel on rails is taken to have: nose on the commanded pointing, not turning.
[[nodiscard]] Attitude pointing_attitude(const vessel::Pointing& pointing,
                                         const orbital::StateVector& state) noexcept;

} // namespace detail

} // namespace helios::sim

#endif // HELIOS_SOURCE_SIM_BUBBLE_HPP
