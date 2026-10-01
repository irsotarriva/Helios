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

#include <cstddef>
#include <optional>

namespace helios::sim {

// The physics bubble (BRIEFING §5.3, D25): the one vessel that is simulated as a rigid body.
struct Simulation::Bubble {
    physics::World world;
    std::size_t occupant = 0; // index of the vessel
    // Where the occupant's propagator and attitude stand: the start of the next physics tick,
    // at most one tick behind the simulation clock.
    time::Epoch epoch;
    std::optional<physics::BodyId> body;
    std::optional<physics::BodyId> ground;
    // What the body was built from; it is rebuilt when the vessel no longer matches.
    math::Vector3 body_centre_of_mass_m; // vessel axes
    std::size_t body_part_count = 0;
    double impact_tolerance_m_s = 0.0;
    bool touching = false; // during the latest tick
    bool pushed = false;   // something other than gravity acted since the caches were refreshed
    int ticks_at_rest = 0;
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
