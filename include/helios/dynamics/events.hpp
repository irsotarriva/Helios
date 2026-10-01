#ifndef HELIOS_DYNAMICS_EVENTS_HPP
#define HELIOS_DYNAMICS_EVENTS_HPP

#include "helios/bodies/body_catalog.hpp"
#include "helios/core/error.hpp"
#include "helios/dynamics/encke_propagator.hpp"
#include "helios/dynamics/gravity_model.hpp"
#include "helios/time/epoch.hpp"

#include <cstdint>
#include <string_view>
#include <vector>

namespace helios::dynamics {

enum class EventKind : std::uint8_t {
    Periapsis,
    Apoapsis,
    DomainExit,    // leaving the current body's sphere of influence
    DomainEntry,   // entering a child body's sphere of influence
    SurfaceImpact, // reaching the domain body's mean radius
};

[[nodiscard]] std::string_view to_string(EventKind kind) noexcept;

struct PredictedEvent {
    EventKind kind = EventKind::Periapsis;
    time::Epoch epoch;
    bodies::BodyId body; // the child for DomainEntry, otherwise the current domain body
};

struct EventSearchOptions {
    double horizon_s = 0.0;
    // Must match PropagatorOptions::domain_hysteresis, so predicted domain changes coincide with
    // the propagator's.
    double domain_hysteresis = 0.05;
};

// Upcoming events of a vessel, predicted on its osculating conic in its current domain and
// returned in time order. Used to limit time warp (BRIEFING §7.1): the prediction ignores
// perturbations, so it is a schedule for slowing down, not the event itself; the propagator
// still decides when the domain actually changes.
//
// Periapsis, apoapsis, exit and impact are closed-form (orbital/conic.hpp). Entry into a child's
// sphere of influence is found by conservative stepping: from distance d and relative speed v the
// sphere cannot be reached in less than about (d − R)/v, so that is (half) the next step; the
// crossing is then bracketed and bisected. Nothing after the first domain change or impact is
// reported, since the conic no longer applies there.
[[nodiscard]] core::Result<std::vector<PredictedEvent>>
predict_conic_events(const GravityModel& gravity, const VesselState& vessel,
                     const EventSearchOptions& options);

} // namespace helios::dynamics

#endif // HELIOS_DYNAMICS_EVENTS_HPP
