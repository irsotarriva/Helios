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

    [[nodiscard]] BubbleMember* find(std::size_t vessel) noexcept {
        const auto found = std::ranges::find(members, vessel, &BubbleMember::vessel);
        return found != members.end() ? &*found : nullptr;
    }
};

// The inside of the vessel the pilot moves about in (BRIEFING D27): its walls, at rest in the
// vessel's axes, and the pilot's body.
struct Simulation::Cabin {
    physics::World world;
    std::optional<physics::BodyId> body;               // the pilot's, while out of the seat
    std::vector<std::optional<physics::BodyId>> items; // per cabin item; none while it is held
    std::size_t part_count = 0;                        // of the vessel the walls were built from
    time::Epoch epoch;                                 // where the cabin's physics stands
    math::Vector3 spin_rad_s;                          // the vessel's at the latest tick, vessel axes
    bool has_spin = false;
    double heavy_s = 0.0;   // for how long the weight felt has been enough to stand against
    bool grab_held = false; // the grab input at the latest tick: a grip starts when it goes down
    bool jump_held = false;
    math::Vector3 grip_offset_m;              // where the arm wants the body, from the gripping hand
    math::Vector3 grip_normal;                // of the surface held, pointing out of it
    std::optional<std::size_t> item_in_reach; // the loose item the pilot looks at, within reach
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
