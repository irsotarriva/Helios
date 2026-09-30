#ifndef HELIOS_DYNAMICS_ENCKE_PROPAGATOR_HPP
#define HELIOS_DYNAMICS_ENCKE_PROPAGATOR_HPP

#include "helios/bodies/body_catalog.hpp"
#include "helios/core/error.hpp"
#include "helios/dynamics/dormand_prince.hpp"
#include "helios/dynamics/gravity_model.hpp"
#include "helios/orbital/kepler.hpp"
#include "helios/time/epoch.hpp"

#include <cstdint>
#include <functional>
#include <limits>

namespace helios::dynamics {

// Where a vessel is: its domain (anchor body), the instant, and its state in that body's
// non-rotating frame.
struct VesselState {
    bodies::BodyId domain;
    time::Epoch epoch;
    orbital::StateVector state_in_domain;
};

struct PropagatorOptions {
    double relative_tolerance = 1e-12; // of the orbit scale (|ρ|, |ρ̇|)
    double absolute_position_tolerance_m = 1e-4;
    double absolute_velocity_tolerance_m_s = 1e-8;
    double max_step_s = std::numeric_limits<double>::infinity();
    double min_step_s = 1e-3;
    double rectification_ratio = 1e-3; // re-osculate when |δ| > ratio · |ρ|
    bool switch_domains = true;
    double domain_hysteresis = 0.05; // leave at (1+h)·R_SOI, enter at (1−h)·R_SOI
};

struct PropagatorStatistics {
    std::uint64_t accepted_steps = 0;
    std::uint64_t rejected_steps = 0;
    std::uint64_t rectifications = 0;
    std::uint64_t domain_changes = 0;
};

// Encke's method (Battin §9.3): integrate only the deviation δ = r − ρ from an osculating
// two-body conic ρ(t) around the domain body, with an adaptive Dormand–Prince 5(4) integrator.
//
// Warp invariance (BRIEFING §7.1, D14): the step sequence depends only on the dynamics and the
// tolerances, never on the instants at which state_at() is called. Samples inside a completed
// step are interpolated (the exact conic plus a quintic Hermite of δ), and rectification or a
// domain change only happens at step boundaries. Sampling the same span at 1 s or at 1e6 s
// cadence therefore yields bit-identical trajectories.
class EnckePropagator {
public:
    [[nodiscard]] static core::Result<EnckePropagator>
    make(const GravityModel& gravity, const VesselState& initial, PropagatorOptions options);

    // State at `instant` ≥ the start of the current step (forward-only; the past is committed
    // to worldlines, not kept here).
    [[nodiscard]] core::Result<VesselState> state_at(const time::Epoch& instant);

    [[nodiscard]] const PropagatorStatistics& statistics() const noexcept { return statistics_; }
    [[nodiscard]] bodies::BodyId current_domain() const noexcept { return domain_; }

private:
    EnckePropagator(const GravityModel& gravity, PropagatorOptions options) noexcept
        : gravity_(gravity), options_(options) {}

    [[nodiscard]] core::VoidResult rebase(bodies::BodyId domain, const time::Epoch& epoch,
                                          const orbital::StateVector& state);
    [[nodiscard]] core::Result<State6> derivative(double time_s, const State6& deviation) const noexcept;
    [[nodiscard]] core::VoidResult take_step();
    [[nodiscard]] core::VoidResult apply_pending_boundary_events();
    [[nodiscard]] core::Result<orbital::StateVector> full_state(double time_s,
                                                                const State6& deviation) const noexcept;
    [[nodiscard]] core::Result<orbital::StateVector> interpolate(double time_s) const noexcept;

    std::reference_wrapper<const GravityModel> gravity_;
    PropagatorOptions options_;
    PropagatorStatistics statistics_;

    // Reference conic, valid for the current segment.
    bodies::BodyId domain_;
    double central_mu_m3_s2_ = 0.0;
    time::Epoch reference_epoch_;
    orbital::StateVector reference_state_;

    // Current step [start, end], local seconds since reference_epoch_.
    double step_start_s_ = 0.0;
    double step_end_s_ = 0.0;
    State6 deviation_start_{};
    State6 deviation_rate_start_{};
    State6 deviation_end_{};
    State6 deviation_rate_end_{};
    double next_step_s_ = 0.0;
    bool boundary_check_pending_ = false;
};

// Cowell's method: integrate the full r̈ directly, in a fixed domain. A deliberately independent
// formulation, used as the reference that Encke is validated against.
[[nodiscard]] core::Result<orbital::StateVector>
propagate_cowell(const GravityModel& gravity, bodies::BodyId domain, const time::Epoch& start,
                 const orbital::StateVector& initial, double duration_s, double relative_tolerance,
                 double max_step_s);

} // namespace helios::dynamics

#endif // HELIOS_DYNAMICS_ENCKE_PROPAGATOR_HPP
