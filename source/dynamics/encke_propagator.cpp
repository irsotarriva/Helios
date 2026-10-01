#include "helios/dynamics/encke_propagator.hpp"

#include "helios/orbital/conic.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>

namespace helios::dynamics {

namespace {

using core::ErrorCode;
using math::Vector3;

[[nodiscard]] Vector3 position_part(const State6& state) noexcept {
    return {state[0], state[1], state[2]};
}
[[nodiscard]] Vector3 velocity_part(const State6& state) noexcept {
    return {state[3], state[4], state[5]};
}
[[nodiscard]] State6 join(const Vector3& position, const Vector3& velocity) noexcept {
    return {position.x, position.y, position.z, velocity.x, velocity.y, velocity.z};
}

// Battin's f(q) = 1 − (1 + q)^(−3/2), evaluated without cancellation for small q.
[[nodiscard]] double encke_f(double q) noexcept {
    const double power = std::pow(1.0 + q, 1.5);
    return q * (3.0 + 3.0 * q + q * q) / ((power + 1.0) * power);
}

struct HermiteSample {
    Vector3 value;
    Vector3 rate;
};

// Quintic Hermite through value, first and second derivative at both ends of [0, h].
[[nodiscard]] HermiteSample quintic_hermite(const Vector3& value_start, const Vector3& rate_start,
                                            const Vector3& curvature_start, const Vector3& value_end,
                                            const Vector3& rate_end, const Vector3& curvature_end,
                                            double step_s, double fraction) noexcept {
    const double s = fraction;
    const double s2 = s * s;
    const double s3 = s2 * s;
    const double s4 = s3 * s;
    const double s5 = s4 * s;
    const double h0 = 1.0 - 10.0 * s3 + 15.0 * s4 - 6.0 * s5;
    const double h1 = s - 6.0 * s3 + 8.0 * s4 - 3.0 * s5;
    const double h2 = 0.5 * s2 - 1.5 * s3 + 1.5 * s4 - 0.5 * s5;
    const double h3 = 0.5 * s3 - s4 + 0.5 * s5;
    const double h4 = -4.0 * s3 + 7.0 * s4 - 3.0 * s5;
    const double h5 = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
    const double d0 = -30.0 * s2 + 60.0 * s3 - 30.0 * s4;
    const double d1 = 1.0 - 18.0 * s2 + 32.0 * s3 - 15.0 * s4;
    const double d2 = s - 4.5 * s2 + 6.0 * s3 - 2.5 * s4;
    const double d3 = 1.5 * s2 - 4.0 * s3 + 2.5 * s4;
    const double d4 = -12.0 * s2 + 28.0 * s3 - 15.0 * s4;
    const double d5 = 30.0 * s2 - 60.0 * s3 + 30.0 * s4;
    const double h = step_s;
    const Vector3 value = h0 * value_start + h1 * h * rate_start + h2 * h * h * curvature_start
                          + h3 * h * h * curvature_end + h4 * h * rate_end + h5 * value_end;
    const Vector3 rate_times_h = d0 * value_start + d1 * h * rate_start + d2 * h * h * curvature_start
                                 + d3 * h * h * curvature_end + d4 * h * rate_end + d5 * value_end;
    return HermiteSample{.value = value, .rate = rate_times_h / h};
}

[[nodiscard]] double orbital_period_estimate_s(const orbital::StateVector& state, double mu_m3_s2) noexcept {
    const double radius_m = math::norm(state.position_m);
    return orbital::k_two_pi * std::sqrt(radius_m * radius_m * radius_m / mu_m3_s2);
}

// The analytic regime samples the perturbation at a few points only (and tides are up to twice
// as strong along the line to the perturber as across it), so it keeps this margin below the
// configured ratio.
constexpr double k_analytic_safety_factor = 2.0;
// Instants across a segment at which the perturbers are sampled; they move slowly compared
// with the segment (days against the Moon's month).
constexpr int k_analytic_instants = 5;
// Apsides and the two points a quarter of the way round: where tides are extreme.
constexpr std::array<double, 4> k_analytic_true_anomalies_rad{0.0, 0.5 * orbital::k_pi, orbital::k_pi,
                                                              1.5 * orbital::k_pi};

} // namespace

core::Result<EnckePropagator> EnckePropagator::make(const GravityModel& gravity, const VesselState& initial,
                                                    PropagatorOptions options) {
    if (!(options.relative_tolerance > 0.0) || !(options.absolute_position_tolerance_m > 0.0)
        || !(options.absolute_velocity_tolerance_m_s > 0.0) || !(options.max_step_s > 0.0)
        || !(options.min_step_s > 0.0) || !(options.rectification_ratio > 0.0)
        || options.domain_hysteresis < 0.0 || options.domain_hysteresis >= 1.0
        || !(options.analytic_perturbation_ratio >= 0.0) || !(options.analytic_segment_s > 0.0)
        || !std::isfinite(options.analytic_segment_s)) {
        return core::fail(ErrorCode::OutOfRange, "invalid propagator options");
    }
    EnckePropagator propagator(gravity, options);
    if (core::VoidResult based = propagator.rebase(initial.domain, initial.epoch, initial.state_in_domain);
        !based) {
        return std::unexpected(based.error());
    }
    propagator.next_step_s_ =
        std::min(options.max_step_s,
                 0.01 * orbital_period_estimate_s(initial.state_in_domain, propagator.central_mu_m3_s2_));
    propagator.latest_returned_ = initial.epoch;
    return propagator;
}

core::VoidResult EnckePropagator::rebase(bodies::BodyId domain, const time::Epoch& epoch,
                                         const orbital::StateVector& state) {
    const auto body = gravity_.get().catalog().body(domain);
    if (!body) {
        return std::unexpected(body.error());
    }
    if (body->get().gravitational_parameter_m3_s2 <= 0.0) {
        return core::fail(ErrorCode::InvalidArgument, "a vessel's domain body must have a positive μ");
    }
    domain_ = domain;
    central_mu_m3_s2_ = body->get().gravitational_parameter_m3_s2;
    reference_epoch_ = epoch;
    reference_state_ = state;
    step_start_s_ = 0.0;
    step_end_s_ = 0.0;
    deviation_start_ = State6{};
    deviation_end_ = State6{};
    const auto rate = derivative(0.0, deviation_end_);
    if (!rate) {
        return std::unexpected(rate.error());
    }
    deviation_rate_start_ = *rate;
    deviation_rate_end_ = *rate;
    boundary_check_pending_ = false;
    analytic_ = false;
    next_analytic_check_s_ = 0.0;
    return {};
}

core::Result<bool> EnckePropagator::analytic_segment_allowed(const orbital::StateVector& state,
                                                             const time::Epoch& start) const {
    // Only a bound orbit that can neither leave its domain nor reach a child's sphere of
    // influence: an analytic segment has no step boundaries at which to change domain.
    const auto conic = orbital::conic_geometry(state, central_mu_m3_s2_);
    if (!conic || !conic->is_bound()) {
        return false;
    }
    const frames::FrameTree& tree = gravity_.get().tree();
    const bodies::BodyCatalog& catalog = gravity_.get().catalog();
    const bodies::Body& current = catalog.body(domain_)->get();
    if (current.domain_parent.has_value() && conic->apoapsis_radius_m >= current.domain_radius_m) {
        return false;
    }
    std::array<Vector3, k_analytic_true_anomalies_rad.size()> points_m;
    for (std::size_t index = 0; index < points_m.size(); ++index) {
        points_m.at(index) =
            orbital::conic_position_at_true_anomaly(*conic, k_analytic_true_anomalies_rad.at(index));
    }
    for (int sample = 0; sample < k_analytic_instants; ++sample) {
        const auto instant =
            start.advanced_by(options_.analytic_segment_s * sample / (k_analytic_instants - 1));
        if (!instant) {
            return std::unexpected(instant.error());
        }
        for (const bodies::BodyId child : catalog.domain_children(domain_)) {
            const bodies::Body& child_body = catalog.body(child)->get();
            const auto child_offset = tree.relative_state(child_body.frame, current.frame, *instant);
            if (!child_offset) {
                return std::unexpected(child_offset.error());
            }
            if (math::norm(child_offset->position_m) - conic->apoapsis_radius_m
                <= child_body.domain_radius_m) {
                return false;
            }
        }
        for (const Vector3& point_m : points_m) {
            const auto perturbation = gravity_.get().perturbing_acceleration_m_s2(domain_, point_m, *instant);
            if (!perturbation) {
                return std::unexpected(perturbation.error());
            }
            const double central_m_s2 = central_mu_m3_s2_ / math::squared_norm(point_m);
            if (k_analytic_safety_factor * math::norm(*perturbation)
                > options_.analytic_perturbation_ratio * central_m_s2) {
                return false;
            }
        }
    }
    return true;
}

core::Result<bool> EnckePropagator::try_enter_analytic(const orbital::StateVector& state,
                                                       const time::Epoch& instant) {
    // Whatever the outcome, do not look again before the perturbers or the orbit have changed.
    next_analytic_check_s_ =
        step_end_s_
        + std::min(orbital_period_estimate_s(state, central_mu_m3_s2_), options_.analytic_segment_s);
    // One evaluation where the vessel is rules out most candidates cheaply.
    const auto here = gravity_.get().perturbing_acceleration_m_s2(domain_, state.position_m, instant);
    if (!here) {
        return std::unexpected(here.error());
    }
    if (k_analytic_safety_factor * math::norm(*here)
        > options_.analytic_perturbation_ratio * central_mu_m3_s2_ / math::squared_norm(state.position_m)) {
        return false;
    }
    const auto allowed = analytic_segment_allowed(state, instant);
    if (!allowed) {
        return std::unexpected(allowed.error());
    }
    if (!*allowed) {
        return false;
    }
    // The conic of the regime is the one osculating the integrated state here.
    if (core::VoidResult based = rebase(domain_, instant, state); !based) {
        return std::unexpected(based.error());
    }
    begin_analytic_segment();
    return true;
}

void EnckePropagator::begin_analytic_segment() noexcept {
    analytic_ = true;
    step_start_s_ = step_end_s_;
    step_end_s_ += options_.analytic_segment_s;
    deviation_start_ = State6{};
    deviation_rate_start_ = State6{};
    deviation_end_ = State6{};
    deviation_rate_end_ = State6{};
    boundary_check_pending_ = false;
    ++statistics_.analytic_segments;
}

core::Result<State6> EnckePropagator::derivative(double time_s, const State6& deviation) const noexcept {
    const auto reference = orbital::propagate_conic(reference_state_, central_mu_m3_s2_, time_s);
    if (!reference) {
        return std::unexpected(reference.error());
    }
    const auto instant = reference_epoch_.advanced_by(time_s);
    if (!instant) {
        return std::unexpected(instant.error());
    }
    const Vector3 delta_m = position_part(deviation);
    const Vector3& rho_m = reference->position_m;
    const Vector3 position_m = rho_m + delta_m;
    const auto perturbation = gravity_.get().perturbing_acceleration_m_s2(domain_, position_m, *instant);
    if (!perturbation) {
        return std::unexpected(perturbation.error());
    }
    const double rho_squared_m2 = math::squared_norm(rho_m);
    const double rho_cubed_m3 = rho_squared_m2 * std::sqrt(rho_squared_m2);
    const double q = math::dot(delta_m, delta_m + 2.0 * rho_m) / rho_squared_m2;
    const Vector3 delta_acceleration_m_s2 =
        (central_mu_m3_s2_ / rho_cubed_m3) * (encke_f(q) * position_m - delta_m) + *perturbation;
    return join(velocity_part(deviation), delta_acceleration_m_s2);
}

core::Result<orbital::StateVector> EnckePropagator::full_state(double time_s,
                                                               const State6& deviation) const noexcept {
    return orbital::propagate_conic(reference_state_, central_mu_m3_s2_, time_s)
        .transform([&](const orbital::StateVector& reference) {
            return orbital::StateVector{.position_m = reference.position_m + position_part(deviation),
                                        .velocity_m_s = reference.velocity_m_s + velocity_part(deviation)};
        });
}

core::Result<orbital::StateVector> EnckePropagator::interpolate(double time_s) const noexcept {
    const double step_s = step_end_s_ - step_start_s_;
    if (step_s == 0.0 || analytic_) {
        return full_state(time_s, deviation_end_);
    }
    const double fraction = std::clamp((time_s - step_start_s_) / step_s, 0.0, 1.0);
    const HermiteSample delta =
        quintic_hermite(position_part(deviation_start_), velocity_part(deviation_start_),
                        velocity_part(deviation_rate_start_), position_part(deviation_end_),
                        velocity_part(deviation_end_), velocity_part(deviation_rate_end_), step_s, fraction);
    return full_state(time_s, join(delta.value, delta.rate));
}

core::VoidResult EnckePropagator::take_step() {
    if (analytic_) {
        // The end of an analytic segment: continue on the same conic if the next segment
        // qualifies, otherwise resume integrating from the conic's state here.
        const auto state = full_state(step_end_s_, State6{});
        const auto instant = reference_epoch_.advanced_by(step_end_s_);
        if (!state || !instant) {
            return core::fail(ErrorCode::OutOfRange, "cannot evaluate the state at the segment boundary");
        }
        const auto allowed = analytic_segment_allowed(*state, *instant);
        if (!allowed) {
            return std::unexpected(allowed.error());
        }
        if (*allowed) {
            begin_analytic_segment();
            return {};
        }
        const double carried_step_s = next_step_s_;
        if (core::VoidResult based = rebase(domain_, *instant, *state); !based) {
            return based;
        }
        const double period_s = orbital_period_estimate_s(*state, central_mu_m3_s2_);
        next_step_s_ = std::min(carried_step_s, 0.01 * period_s);
        next_analytic_check_s_ = std::min(period_s, options_.analytic_segment_s);
    }
    const auto reference = orbital::propagate_conic(reference_state_, central_mu_m3_s2_, step_end_s_);
    if (!reference) {
        return std::unexpected(reference.error());
    }
    const double position_scale_m = options_.absolute_position_tolerance_m
                                    + options_.relative_tolerance * math::norm(reference->position_m);
    const double velocity_scale_m_s = options_.absolute_velocity_tolerance_m_s
                                      + options_.relative_tolerance * math::norm(reference->velocity_m_s);
    const State6 scale{position_scale_m,   position_scale_m,   position_scale_m,
                       velocity_scale_m_s, velocity_scale_m_s, velocity_scale_m_s};
    const auto function = [this](double time_s, const State6& deviation) {
        return derivative(time_s, deviation);
    };

    double step_s = std::min(next_step_s_, options_.max_step_s);
    for (;;) {
        const auto attempt =
            dormand_prince_step(function, step_end_s_, deviation_end_, deviation_rate_end_, step_s, scale);
        if (!attempt) {
            return std::unexpected(attempt.error());
        }
        const double factor = next_step_factor(attempt->error_norm);
        if (attempt->error_norm <= 1.0) {
            step_start_s_ = step_end_s_;
            deviation_start_ = deviation_end_;
            deviation_rate_start_ = deviation_rate_end_;
            step_end_s_ += step_s;
            deviation_end_ = attempt->state;
            deviation_rate_end_ = attempt->derivative;
            next_step_s_ = std::min(step_s * factor, options_.max_step_s);
            boundary_check_pending_ = true;
            ++statistics_.accepted_steps;
            return {};
        }
        ++statistics_.rejected_steps;
        step_s *= std::max(factor, 0.2);
        if (step_s < options_.min_step_s) {
            return core::fail(
                ErrorCode::OutOfRange,
                std::format("step size fell below {} s (collision or singularity?)", options_.min_step_s));
        }
    }
}

core::VoidResult EnckePropagator::apply_pending_boundary_events() {
    boundary_check_pending_ = false;
    const auto state = full_state(step_end_s_, deviation_end_);
    const auto instant = reference_epoch_.advanced_by(step_end_s_);
    if (!state || !instant) {
        return core::fail(ErrorCode::OutOfRange, "cannot evaluate the state at the step boundary");
    }

    if (options_.switch_domains) {
        const frames::FrameTree& tree = gravity_.get().tree();
        const bodies::BodyCatalog& catalog = gravity_.get().catalog();
        const bodies::Body& current = catalog.body(domain_)->get();
        std::optional<bodies::BodyId> destination;
        if (current.domain_parent.has_value()
            && math::norm(state->position_m) > current.domain_radius_m * (1.0 + options_.domain_hysteresis)) {
            destination = current.domain_parent;
        } else {
            for (const bodies::BodyId child : catalog.domain_children(domain_)) {
                const bodies::Body& child_body = catalog.body(child)->get();
                const auto child_offset = tree.relative_state(child_body.frame, current.frame, *instant);
                if (!child_offset) {
                    return std::unexpected(child_offset.error());
                }
                if (math::norm(state->position_m - child_offset->position_m)
                    < child_body.domain_radius_m * (1.0 - options_.domain_hysteresis)) {
                    destination = child;
                    break;
                }
            }
        }
        if (destination.has_value()) {
            const bodies::Body& target = catalog.body(*destination)->get();
            const auto offset = tree.relative_state(current.frame, target.frame, *instant);
            if (!offset) {
                return std::unexpected(offset.error());
            }
            const orbital::StateVector in_destination{.position_m = state->position_m + offset->position_m,
                                                      .velocity_m_s =
                                                          state->velocity_m_s + offset->velocity_m_s};
            ++statistics_.domain_changes;
            const double carried_step_s = next_step_s_;
            if (core::VoidResult based = rebase(*destination, *instant, in_destination); !based) {
                return based;
            }
            next_step_s_ =
                std::min(carried_step_s, 0.01 * orbital_period_estimate_s(in_destination, central_mu_m3_s2_));
            return {};
        }
    }

    if (options_.analytic_perturbation_ratio > 0.0 && step_end_s_ >= next_analytic_check_s_) {
        const auto entered = try_enter_analytic(*state, *instant);
        if (!entered) {
            return std::unexpected(entered.error());
        }
        if (*entered) {
            return {};
        }
    }

    const auto reference = orbital::propagate_conic(reference_state_, central_mu_m3_s2_, step_end_s_);
    if (!reference) {
        return std::unexpected(reference.error());
    }
    if (math::norm(position_part(deviation_end_))
        > options_.rectification_ratio * math::norm(reference->position_m)) {
        ++statistics_.rectifications;
        const double carried_step_s = next_step_s_;
        if (core::VoidResult based = rebase(domain_, *instant, *state); !based) {
            return based;
        }
        next_step_s_ = carried_step_s;
    }
    return {};
}

core::VoidResult EnckePropagator::schedule_impulse(const Impulse& impulse) {
    const Vector3& delta_v_m_s = impulse.delta_v_m_s;
    if (!std::isfinite(delta_v_m_s.x) || !std::isfinite(delta_v_m_s.y) || !std::isfinite(delta_v_m_s.z)) {
        return core::fail(ErrorCode::NotFinite, "impulse Δv is not finite");
    }
    if (impulse.epoch < latest_returned_) {
        return core::fail(ErrorCode::OutOfRange,
                          "cannot schedule an impulse before an instant already returned");
    }
    const auto position = std::ranges::upper_bound(impulses_, impulse.epoch, std::less{}, &Impulse::epoch);
    impulses_.insert(position, impulse);
    return {};
}

core::VoidResult EnckePropagator::apply_next_impulse() {
    const Impulse impulse = impulses_.front();
    impulses_.erase(impulses_.begin());
    const auto state = interpolate(time::seconds_between(reference_epoch_, impulse.epoch));
    if (!state) {
        return std::unexpected(state.error());
    }
    const orbital::StateVector after{.position_m = state->position_m,
                                     .velocity_m_s = state->velocity_m_s + impulse.delta_v_m_s};
    ++statistics_.impulses;
    const double carried_step_s = next_step_s_;
    if (core::VoidResult based = rebase(domain_, impulse.epoch, after); !based) {
        return based;
    }
    next_step_s_ = std::min(carried_step_s, 0.01 * orbital_period_estimate_s(after, central_mu_m3_s2_));
    return {};
}

core::Result<VesselState> EnckePropagator::state_at(const time::Epoch& instant) {
    for (;;) {
        const double time_s = time::seconds_between(reference_epoch_, instant);
        if (time_s < step_start_s_) {
            return core::fail(ErrorCode::OutOfRange,
                              "the propagator is forward-only; that instant has passed");
        }
        if (!impulses_.empty()) {
            const double impulse_s = time::seconds_between(reference_epoch_, impulses_.front().epoch);
            if (impulse_s <= step_end_s_ && impulse_s <= time_s) {
                if (core::VoidResult applied = apply_next_impulse(); !applied) {
                    return std::unexpected(applied.error());
                }
                continue;
            }
        }
        if (time_s <= step_end_s_) {
            return interpolate(time_s).transform([&](const orbital::StateVector& state) {
                latest_returned_ = std::max(latest_returned_, instant);
                return VesselState{.domain = domain_, .epoch = instant, .state_in_domain = state};
            });
        }
        if (boundary_check_pending_) {
            if (core::VoidResult applied = apply_pending_boundary_events(); !applied) {
                return std::unexpected(applied.error());
            }
            continue;
        }
        if (core::VoidResult stepped = take_step(); !stepped) {
            return std::unexpected(stepped.error());
        }
    }
}

core::Result<orbital::StateVector> propagate_cowell(const GravityModel& gravity, bodies::BodyId domain,
                                                    const time::Epoch& start,
                                                    const orbital::StateVector& initial, double duration_s,
                                                    double relative_tolerance, double max_step_s) {
    const auto function = [&](double time_s, const State6& state) -> core::Result<State6> {
        const auto instant = start.advanced_by(time_s);
        if (!instant) {
            return std::unexpected(instant.error());
        }
        return gravity.total_acceleration_m_s2(domain, position_part(state), *instant)
            .transform([&](const Vector3& acceleration_m_s2) {
                return join(velocity_part(state), acceleration_m_s2);
            });
    };
    State6 state = join(initial.position_m, initial.velocity_m_s);
    auto rate = function(0.0, state);
    if (!rate) {
        return std::unexpected(rate.error());
    }
    double time_s = 0.0;
    double step_s = std::min(max_step_s, 60.0);
    while (time_s < duration_s) {
        step_s = std::min(step_s, duration_s - time_s);
        const double position_scale_m = relative_tolerance * math::norm(position_part(state)) + 1e-9;
        const double velocity_scale_m_s = relative_tolerance * math::norm(velocity_part(state)) + 1e-12;
        const State6 scale{position_scale_m,   position_scale_m,   position_scale_m,
                           velocity_scale_m_s, velocity_scale_m_s, velocity_scale_m_s};
        const auto attempt = dormand_prince_step(function, time_s, state, *rate, step_s, scale);
        if (!attempt) {
            return std::unexpected(attempt.error());
        }
        const double factor = next_step_factor(attempt->error_norm);
        if (attempt->error_norm <= 1.0) {
            time_s += step_s;
            state = attempt->state;
            *rate = attempt->derivative;
        }
        step_s = std::min(step_s * factor, max_step_s);
        if (step_s < 1e-6) {
            return core::fail(ErrorCode::OutOfRange, "Cowell step size collapsed");
        }
    }
    return orbital::StateVector{.position_m = position_part(state), .velocity_m_s = velocity_part(state)};
}

} // namespace helios::dynamics
