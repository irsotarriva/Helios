#include "helios/sim/simulation.hpp"

#include "helios/core/logging.hpp"
#include "helios/orbital/conic.hpp"

#include <algorithm>
#include <cmath>

namespace helios::sim {

namespace {

using core::ErrorCode;

constexpr int k_max_impact_search_steps = 100'000;
constexpr int k_impact_bisections = 40;

// First state on the path of `propagator` (positioned at `from`) in (from, to] that is at or
// below `surface_radius_m`, found by conservative stepping (the surface is at least
// (r − R)/|v| away) and bisection. Uses copies, since propagators only move forward.
[[nodiscard]] std::optional<dynamics::VesselState> locate_impact(const dynamics::EnckePropagator& propagator,
                                                                 const time::Epoch& from,
                                                                 const time::Epoch& to,
                                                                 double surface_radius_m) {
    dynamics::EnckePropagator outside = propagator;
    time::Epoch outside_epoch = from;
    for (int step = 0; step < k_max_impact_search_steps && outside_epoch < to; ++step) {
        const auto outside_state = dynamics::EnckePropagator(outside).state_at(outside_epoch);
        if (!outside_state) {
            return std::nullopt;
        }
        const double radius_m = math::norm(outside_state->state_in_domain.position_m);
        const double speed_m_s = math::norm(outside_state->state_in_domain.velocity_m_s);
        const double stride_s =
            std::max(0.5 * (radius_m - surface_radius_m) / std::max(speed_m_s, 1e-3), 0.1);
        auto candidate_epoch = outside_epoch.advanced_by(stride_s);
        if (!candidate_epoch) {
            return std::nullopt;
        }
        const time::Epoch inside_epoch = std::min(*candidate_epoch, to);
        dynamics::EnckePropagator probe = outside;
        const auto probe_state = probe.state_at(inside_epoch);
        if (!probe_state) {
            return std::nullopt;
        }
        if (math::norm(probe_state->state_in_domain.position_m) > surface_radius_m) {
            outside = std::move(probe);
            outside_epoch = inside_epoch;
            continue;
        }
        // Bracketed: bisect between outside_epoch and inside_epoch.
        dynamics::VesselState inside_state = *probe_state;
        double low_s = 0.0;
        double high_s = time::seconds_between(outside_epoch, inside_epoch);
        for (int iteration = 0; iteration < k_impact_bisections; ++iteration) {
            const double middle_s = 0.5 * (low_s + high_s);
            const auto middle_epoch = outside_epoch.advanced_by(middle_s);
            if (!middle_epoch) {
                break;
            }
            dynamics::EnckePropagator middle = outside;
            const auto middle_state = middle.state_at(*middle_epoch);
            if (!middle_state) {
                break;
            }
            if (math::norm(middle_state->state_in_domain.position_m) > surface_radius_m) {
                low_s = middle_s;
            } else {
                high_s = middle_s;
                inside_state = *middle_state;
            }
        }
        return inside_state;
    }
    return std::nullopt;
}

[[nodiscard]] bool limits_warp(dynamics::EventKind kind) noexcept {
    return kind == dynamics::EventKind::DomainExit || kind == dynamics::EventKind::DomainEntry
           || kind == dynamics::EventKind::SurfaceImpact;
}

} // namespace

Simulation::Simulation(std::unique_ptr<frames::FrameTree> tree, std::unique_ptr<bodies::BodyCatalog> catalog,
                       std::unique_ptr<dynamics::GravityModel> gravity, const time::Epoch& start,
                       SimulationOptions options) noexcept
    : tree_(std::move(tree)), catalog_(std::move(catalog)), gravity_(std::move(gravity)), options_(options),
      now_(start) {
}

core::Result<Simulation> Simulation::make(std::unique_ptr<frames::FrameTree> tree,
                                          std::unique_ptr<bodies::BodyCatalog> catalog,
                                          const time::Epoch& start, SimulationOptions options) {
    if (!tree || !catalog) {
        return core::fail(ErrorCode::InvalidArgument, "a simulation needs a frame tree and a body catalogue");
    }
    if (!(options.event_horizon_s > 0.0) || !(options.unbound_prediction_horizon_s > 0.0)
        || !(options.max_prediction_horizon_s > 0.0) || options.prediction_samples < 2) {
        return core::fail(ErrorCode::OutOfRange, "invalid simulation options");
    }
    auto gravity = dynamics::GravityModel::make(*tree, *catalog, options.gravity, start);
    if (!gravity) {
        return std::unexpected(gravity.error());
    }
    return Simulation(std::move(tree), std::move(catalog),
                      std::make_unique<dynamics::GravityModel>(std::move(*gravity)), start, options);
}

core::Result<VesselId> Simulation::add_vessel(std::string name, const orbital::StateVector& state_in_domain,
                                              bodies::BodyId domain) {
    const dynamics::VesselState initial{.domain = domain, .epoch = now_, .state_in_domain = state_in_domain};
    auto propagator = dynamics::EnckePropagator::make(*gravity_, initial, options_.propagator);
    if (!propagator) {
        return std::unexpected(propagator.error());
    }
    vessels_.push_back(Vessel{.name = std::move(name),
                              .status = VesselStatus::Flying,
                              .propagator = std::move(*propagator),
                              .state = initial,
                              .events = {},
                              .prediction = {},
                              .prediction_start = now_,
                              .prediction_horizon_s = 0.0,
                              .events_stale = true,
                              .prediction_stale = true});
    refresh_caches(vessels_.back());
    return VesselId{static_cast<std::uint32_t>(vessels_.size() - 1)};
}

core::Result<std::reference_wrapper<const Vessel>> Simulation::vessel(VesselId id) const noexcept {
    if (id.index >= vessels_.size()) {
        return core::fail(ErrorCode::OutOfRange, "unknown vessel id");
    }
    return std::cref(vessels_[id.index]);
}

core::VoidResult Simulation::schedule_impulse(VesselId id, const dynamics::Impulse& impulse) {
    if (id.index >= vessels_.size()) {
        return core::fail(ErrorCode::OutOfRange, "unknown vessel id");
    }
    Vessel& vessel = vessels_[id.index];
    if (vessel.status != VesselStatus::Flying) {
        return core::fail(ErrorCode::InvalidArgument, "only a flying vessel can manoeuvre");
    }
    if (core::VoidResult scheduled = vessel.propagator.schedule_impulse(impulse); !scheduled) {
        return scheduled;
    }
    vessel.events_stale = true;
    vessel.prediction_stale = true;
    refresh_caches(vessel);
    return {};
}

core::VoidResult Simulation::schedule_maneuver(VesselId id, const time::Epoch& epoch, double prograde_m_s,
                                               double normal_m_s, double radial_out_m_s) {
    if (id.index >= vessels_.size()) {
        return core::fail(ErrorCode::OutOfRange, "unknown vessel id");
    }
    // A copy runs ahead to the burn; the vessel's own propagator is untouched.
    dynamics::EnckePropagator ahead = vessels_[id.index].propagator;
    const auto state = ahead.state_at(epoch);
    if (!state) {
        return std::unexpected(state.error());
    }
    const auto basis = orbital::maneuver_basis(state->state_in_domain);
    if (!basis) {
        return std::unexpected(basis.error());
    }
    return schedule_impulse(id, dynamics::Impulse{.epoch = epoch,
                                                  .delta_v_m_s = orbital::maneuver_to_inertial(
                                                      *basis, prograde_m_s, normal_m_s, radial_out_m_s)});
}

std::optional<WarpLimit> Simulation::warp_limit() const {
    std::optional<WarpLimit> limit;
    const auto consider = [&](const WarpLimit& candidate) {
        if (candidate.epoch >= now_ && (!limit.has_value() || candidate.epoch < limit->epoch)) {
            limit = candidate;
        }
    };
    for (std::uint32_t index = 0; index < vessels_.size(); ++index) {
        const Vessel& vessel = vessels_[index];
        if (vessel.status != VesselStatus::Flying) {
            continue;
        }
        const auto impulses = vessel.propagator.pending_impulses();
        if (!impulses.empty()) {
            consider(WarpLimit{
                .vessel = {index}, .epoch = impulses.front().epoch, .kind = {}, .body = vessel.state.domain});
        }
        for (const dynamics::PredictedEvent& event : vessel.events) {
            if (limits_warp(event.kind) && event.epoch >= now_) {
                consider(WarpLimit{
                    .vessel = {index}, .epoch = event.epoch, .kind = event.kind, .body = event.body});
                break;
            }
        }
    }
    return limit;
}

core::VoidResult Simulation::advance(double wall_dt_s) {
    const std::optional<WarpLimit> limit = warp_limit();
    const auto step = warp_.advance(
        now_, wall_dt_s, limit.has_value() ? std::optional<time::Epoch>(limit->epoch) : std::nullopt);
    if (!step) {
        return std::unexpected(step.error());
    }
    effective_warp_ = step->factor;
    if (core::VoidResult moved = advance_to(step->epoch); !moved) {
        return moved;
    }
    const bool reached_limit = limit.has_value() && step->epoch == limit->epoch;
    if (reached_limit && (!limit->kind.has_value() || *limit->kind == dynamics::EventKind::SurfaceImpact)
        && warp_.requested_level() > k_real_time_level) {
        warp_.request_level(k_real_time_level);
    }
    return {};
}

core::VoidResult Simulation::advance_to(const time::Epoch& instant) {
    if (instant < now_) {
        return core::fail(ErrorCode::OutOfRange, "the simulation clock only moves forward");
    }
    for (Vessel& vessel : vessels_) {
        if (vessel.status != VesselStatus::Flying) {
            continue;
        }
        const bodies::BodyId domain_before = vessel.state.domain;
        const std::uint64_t impulses_before = vessel.propagator.statistics().impulses;
        const time::Epoch previous_epoch = vessel.state.epoch;
        // Rationale: a frame at high warp can carry a vessel straight through a planet, so the
        // end state alone cannot detect an impact. When one is predicted within the frame (with
        // slack for the conic's error), or the end state is underground, search the path.
        const auto impact_slack_s = [&](const dynamics::PredictedEvent& event) {
            return std::max(60.0, 0.05 * time::seconds_between(previous_epoch, event.epoch));
        };
        const bool impact_expected =
            std::ranges::any_of(vessel.events, [&](const dynamics::PredictedEvent& event) {
                return event.kind == dynamics::EventKind::SurfaceImpact
                       && time::seconds_between(instant, event.epoch) <= impact_slack_s(event);
            });
        const dynamics::EnckePropagator before = vessel.propagator;
        const auto state = vessel.propagator.state_at(instant);
        if (!state) {
            vessel.status = VesselStatus::Crashed;
            LOG_WARN("vessel '{}' lost: {}", vessel.name, core::describe(state.error()))
                .tag("subsystem", "sim");
            continue;
        }
        const auto domain_body = catalog_->body(domain_before);
        if (!domain_body) {
            return std::unexpected(domain_body.error());
        }
        const double surface_radius_m = domain_body->get().mean_radius_m;
        const bool underground = state->domain == domain_before
                                 && math::norm(state->state_in_domain.position_m) <= surface_radius_m;
        if (impact_expected || underground) {
            if (const auto impact = locate_impact(before, previous_epoch, instant, surface_radius_m)) {
                vessel.state = *impact;
                vessel.status = VesselStatus::Crashed;
                vessel.events.clear();
                vessel.prediction.clear();
                LOG_INFO("vessel '{}' reached the surface of {}", vessel.name, domain_body->get().name)
                    .tag("subsystem", "sim");
                continue;
            }
        }
        vessel.state = *state;
        const bool changed =
            state->domain != domain_before || vessel.propagator.statistics().impulses != impulses_before;
        const bool event_passed = !vessel.events.empty() && vessel.events.front().epoch <= instant;
        const bool prediction_aging =
            time::seconds_between(vessel.prediction_start, instant) > 0.25 * vessel.prediction_horizon_s;
        vessel.events_stale = vessel.events_stale || changed || event_passed;
        vessel.prediction_stale = vessel.prediction_stale || changed || prediction_aging;
        refresh_caches(vessel);
    }
    now_ = instant;
    return {};
}

void Simulation::refresh_caches(Vessel& vessel) {
    // Rationale: a degenerate (radial) conic has no events or orbit to predict, and a failed
    // prediction only costs the map view a line; neither stops the simulation.
    if (vessel.events_stale) {
        vessel.events_stale = false;
        auto events = dynamics::predict_conic_events(
            *gravity_, vessel.state,
            dynamics::EventSearchOptions{.horizon_s = options_.event_horizon_s,
                                         .domain_hysteresis = options_.propagator.domain_hysteresis});
        vessel.events = events ? std::move(*events) : std::vector<dynamics::PredictedEvent>{};
    }
    if (!vessel.prediction_stale) {
        return;
    }
    vessel.prediction_stale = false;
    double horizon_s = options_.unbound_prediction_horizon_s;
    if (const auto domain = catalog_->body(vessel.state.domain)) {
        if (const auto conic = orbital::conic_geometry(vessel.state.state_in_domain,
                                                       domain->get().gravitational_parameter_m3_s2);
            conic && conic->is_bound()) {
            horizon_s = orbital::orbital_period_s(*conic);
        }
    }
    horizon_s = std::min(horizon_s, options_.max_prediction_horizon_s);
    auto prediction = dynamics::predict_trajectory(
        vessel.propagator, vessel.state.epoch,
        dynamics::PredictionOptions{.horizon_s = horizon_s, .max_samples = options_.prediction_samples});
    vessel.prediction = prediction ? std::move(*prediction) : std::vector<dynamics::TrajectorySegment>{};
    vessel.prediction_start = vessel.state.epoch;
    vessel.prediction_horizon_s = horizon_s;
}

} // namespace helios::sim
