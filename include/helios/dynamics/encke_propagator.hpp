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
#include <span>
#include <vector>

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

    // The analytic regime (BRIEFING §5.3). When the perturbing acceleration stays below this
    // fraction of the central gravity for a whole segment, the vessel follows its osculating
    // conic exactly for that segment instead of being integrated, at a cost that does not
    // depend on how many orbits the segment spans. 0 disables the regime (pure Encke).
    double analytic_perturbation_ratio = 0.0;
    // Length of one analytic segment: how far ahead the perturbations are checked, and so how
    // often the regime is reconsidered.
    double analytic_segment_s = 10.0 * 86'400.0;
};

struct PropagatorStatistics {
    std::uint64_t accepted_steps = 0;
    std::uint64_t rejected_steps = 0;
    std::uint64_t rectifications = 0;
    std::uint64_t domain_changes = 0;
    std::uint64_t impulses = 0;
    std::uint64_t analytic_segments = 0;
};

// An instantaneous velocity change (an impulsive manoeuvre). All frames of the tree share the
// same inertial axes and differ only by a translation, so Δv is the same vector in every domain.
struct Impulse {
    time::Epoch epoch;
    math::Vector3 delta_v_m_s;
};

// Encke's method (Battin §9.3): integrate only the deviation δ = r − ρ from an osculating
// two-body conic ρ(t) around the domain body, with an adaptive Dormand–Prince 5(4) integrator.
//
// Warp invariance (BRIEFING §7.1, D14): the step sequence depends only on the dynamics and the
// tolerances, never on the instants at which state_at() is called. Samples inside a completed
// step are interpolated (the exact conic plus a quintic Hermite of δ), and rectification or a
// domain change only happens at step boundaries. Sampling the same span at 1 s or at 1e6 s
// cadence therefore yields bit-identical trajectories.
//
// The analytic regime keeps that property: whether a segment is analytic is decided at a step
// or segment boundary from the dynamics alone (never from the sampling instants or from
// impulses still pending), and an analytic segment behaves like one long step with zero
// deviation, which an impulse truncates like any other step.
class EnckePropagator {
public:
    [[nodiscard]] static core::Result<EnckePropagator>
    make(const GravityModel& gravity, const VesselState& initial, PropagatorOptions options);

    // State at `instant` ≥ the start of the current step (forward-only; the past is committed
    // to worldlines, not kept here). At an impulse's epoch the post-impulse state is returned.
    [[nodiscard]] core::Result<VesselState> state_at(const time::Epoch& instant);

    // Schedules an impulse. Fails if `impulse.epoch` precedes the latest instant state_at() has
    // returned (that would rewrite a past the caller has already seen) or Δv is not finite.
    //
    // Warp invariance: an impulse never shortens a step. The step that contains it is completed
    // as usual and then truncated at the impulse epoch using its own dense output, after which
    // the conic is re-osculated with the new velocity. The step sequence up to the impulse is
    // therefore the same whether the impulse was scheduled long before or while the propagator
    // was already inside that step, and the trajectory is bit-identical in both cases.
    [[nodiscard]] core::VoidResult schedule_impulse(const Impulse& impulse);

    // Scheduled impulses not yet applied, in epoch order.
    [[nodiscard]] std::span<const Impulse> pending_impulses() const noexcept { return impulses_; }

    [[nodiscard]] const PropagatorStatistics& statistics() const noexcept { return statistics_; }
    [[nodiscard]] bodies::BodyId current_domain() const noexcept { return domain_; }
    // The end of the current integration step or analytic segment. Domain changes only happen
    // at such boundaries, so a vessel that is already across a sphere of influence switches no
    // earlier than this instant, and at it if the step has not been closed yet.
    [[nodiscard]] core::Result<time::Epoch> step_end() const noexcept {
        return reference_epoch_.advanced_by(step_end_s_);
    }
    // Whether the current segment is analytic (an exact conic) rather than integrated.
    [[nodiscard]] bool is_analytic() const noexcept { return analytic_; }
    [[nodiscard]] const GravityModel& gravity() const noexcept { return gravity_.get(); }

private:
    EnckePropagator(const GravityModel& gravity, PropagatorOptions options) noexcept
        : gravity_(gravity), options_(options) {}

    [[nodiscard]] core::VoidResult rebase(bodies::BodyId domain, const time::Epoch& epoch,
                                          const orbital::StateVector& state);
    [[nodiscard]] core::Result<State6> derivative(double time_s, const State6& deviation) const noexcept;
    [[nodiscard]] core::VoidResult take_step();
    [[nodiscard]] core::VoidResult apply_pending_boundary_events();
    [[nodiscard]] core::VoidResult apply_next_impulse();
    [[nodiscard]] core::Result<bool> analytic_segment_allowed(const orbital::StateVector& state,
                                                              const time::Epoch& start) const;
    [[nodiscard]] core::Result<bool> try_enter_analytic(const orbital::StateVector& state,
                                                        const time::Epoch& instant);
    void begin_analytic_segment() noexcept;
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

    bool analytic_ = false;
    double next_analytic_check_s_ = 0.0; // local time of the next attempt to enter the regime

    std::vector<Impulse> impulses_; // sorted by epoch
    time::Epoch latest_returned_;
};

// Cowell's method: integrate the full r̈ directly, in a fixed domain. A deliberately independent
// formulation, used as the reference that Encke is validated against.
[[nodiscard]] core::Result<orbital::StateVector>
propagate_cowell(const GravityModel& gravity, bodies::BodyId domain, const time::Epoch& start,
                 const orbital::StateVector& initial, double duration_s, double relative_tolerance,
                 double max_step_s);

} // namespace helios::dynamics

#endif // HELIOS_DYNAMICS_ENCKE_PROPAGATOR_HPP
