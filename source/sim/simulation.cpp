#include "helios/sim/simulation.hpp"

#include "helios/core/logging.hpp"
#include "helios/orbital/conic.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <utility>

#include "bubble.hpp"

namespace helios::sim {

namespace {

using core::ErrorCode;
using detail::sync_propulsion;
using detail::to_thrust_direction;

constexpr int k_max_impact_search_steps = 100'000;
// How many changes of thrust ahead the propagator is told about. The plan is renewed whenever
// the vessel's systems change, so this only bounds a single frame.
constexpr std::size_t k_max_planned_thrust_changes = 64;

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

dynamics::ThrustDirection detail::to_thrust_direction(const vessel::Pointing& pointing) noexcept {
    dynamics::ThrustDirection::Frame frame = dynamics::ThrustDirection::Frame::Orbital;
    if (pointing.frame == vessel::Pointing::Frame::Inertial) {
        frame = dynamics::ThrustDirection::Frame::Inertial;
    } else if (pointing.frame == vessel::Pointing::Frame::Local) {
        frame = dynamics::ThrustDirection::Frame::Local;
    }
    return {.frame = frame, .direction = pointing.direction};
}

core::VoidResult detail::sync_propulsion(Vessel& vessel, const time::Epoch& from, bool include_current) {
    if (!vessel.systems.has_value()) {
        return {};
    }
    const auto forecast = vessel.systems->forecast(k_max_planned_thrust_changes);
    if (!forecast) {
        return std::unexpected(forecast.error());
    }
    std::vector<dynamics::ThrustChange> changes;
    // A state that only continues a coast is not a change for the trajectory, and listing it
    // would hold the time warp back for nothing.
    double thrust_before_n = vessel.propagator.thrust().thrust_n;
    for (const vessel::PropulsionState& state : *forecast) {
        const bool current = include_current && state.epoch < from && &state == &forecast->front();
        if ((state.epoch >= from || current) && (thrust_before_n != 0.0 || state.thrust_n != 0.0)) {
            thrust_before_n = state.thrust_n;
            // The thrust in effect is handed over as of `from`, with the mass the vessel has then.
            changes.push_back(
                dynamics::ThrustChange{.epoch = current ? from : state.epoch,
                                       .thrust_n = state.thrust_n,
                                       .mass_flow_kg_s = state.mass_flow_kg_s,
                                       .mass_kg = current ? vessel.systems->mass_kg(from) : state.mass_kg,
                                       .dry_mass_kg = state.dry_mass_kg,
                                       .direction = to_thrust_direction(state.pointing)});
        }
    }
    if (std::ranges::equal(changes, vessel.propagator.pending_thrust_changes())) {
        return {};
    }
    vessel.events_stale = true;
    vessel.prediction_stale = true;
    return vessel.propagator.set_thrust_plan(std::move(changes));
}

Simulation::Simulation(std::unique_ptr<frames::FrameTree> tree, std::unique_ptr<bodies::BodyCatalog> catalog,
                       std::unique_ptr<dynamics::GravityModel> gravity, const time::Epoch& start,
                       SimulationOptions options) noexcept
    : tree_(std::move(tree)), catalog_(std::move(catalog)), gravity_(std::move(gravity)), options_(options),
      now_(start) {
    std::size_t max_level = k_real_time_level;
    while (max_level + 1 < k_warp_levels.size()
           && k_warp_levels.at(max_level + 1) <= options.max_warp_factor) {
        ++max_level;
    }
    warp_.set_max_level(max_level);
}

core::Result<Simulation> Simulation::make(std::unique_ptr<frames::FrameTree> tree,
                                          std::unique_ptr<bodies::BodyCatalog> catalog,
                                          const time::Epoch& start, SimulationOptions options) {
    if (!tree || !catalog) {
        return core::fail(ErrorCode::InvalidArgument, "a simulation needs a frame tree and a body catalogue");
    }
    if (!(options.event_horizon_s > 0.0) || !(options.unbound_prediction_horizon_s > 0.0)
        || !(options.max_prediction_horizon_s > 0.0) || options.prediction_samples < 2
        || !(options.max_warp_factor >= 1.0)) {
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
                              .systems = {},
                              .events = {},
                              .events_epoch = now_,
                              .prediction = {},
                              .prediction_start = now_,
                              .prediction_horizon_s = 0.0,
                              .events_stale = true,
                              .prediction_stale = true});
    refresh_caches(vessels_.back());
    return VesselId{static_cast<std::uint32_t>(vessels_.size() - 1)};
}

core::Result<VesselId> Simulation::add_vessel(std::string name, const orbital::StateVector& state_in_domain,
                                              bodies::BodyId domain, vessel::VesselSystems systems) {
    if (systems.propulsion().epoch > now_) {
        return core::fail(ErrorCode::OutOfRange, "the vessel's systems are ahead of the simulation clock");
    }
    std::vector<vessel::Separation> separations;
    if (core::VoidResult advanced = systems.advance_to(now_, separations); !advanced) {
        return std::unexpected(advanced.error());
    }
    // Rationale: the propagator is told of thrust from the current epoch on, so a burn that began
    // before the vessel was added would have no start.
    if (!separations.empty() || (systems.propulsion().thrust_n != 0.0 && systems.propulsion().epoch < now_)) {
        return core::fail(ErrorCode::InvalidArgument,
                          "a vessel cannot be added while it is separating or already under thrust");
    }
    const auto id = add_vessel(std::move(name), state_in_domain, domain);
    if (!id) {
        return id;
    }
    Vessel& vessel = vessels_[id->index];
    vessel.attitude = detail::pointing_attitude(systems.propulsion().pointing, state_in_domain);
    vessel.systems = std::move(systems);
    if (core::VoidResult changed = systems_changed(vessel, now_); !changed) {
        vessels_.pop_back();
        return std::unexpected(changed.error());
    }
    if (core::VoidResult reported = report_navigation(); !reported) {
        return std::unexpected(reported.error());
    }
    return id;
}

core::VoidResult Simulation::report_navigation() {
    for (Vessel& vessel : vessels_) {
        if (!vessel.systems.has_value() || vessel.status == VesselStatus::Crashed
            || vessel.status == VesselStatus::Stowed) {
            continue;
        }
        const auto domain = catalog_->body(vessel.state.domain);
        if (!domain) {
            return std::unexpected(domain.error());
        }
        const bodies::Body& body = domain->get();
        const orbital::StateVector& state = vessel.state.state_in_domain;
        const double radius_m = math::norm(state.position_m);
        const math::Vector3 ground_velocity_m_s =
            body.rotation.has_value()
                ? math::cross(body.rotation->angular_velocity_rad_s(now_), state.position_m)
                : math::Vector3{};
        if (core::VoidResult reported = vessel.systems->report_navigation(
                {.altitude_m = radius_m - body.mean_radius_m,
                 .vertical_speed_m_s =
                     radius_m > 0.0 ? math::dot(state.velocity_m_s, state.position_m) / radius_m : 0.0,
                 .surface_speed_m_s = math::norm(state.velocity_m_s - ground_velocity_m_s),
                 .speed_m_s = math::norm(state.velocity_m_s)});
            !reported) {
            return reported;
        }
    }
    return {};
}

core::Result<std::reference_wrapper<vessel::VesselSystems>> Simulation::systems_of(VesselId id) {
    if (id.index >= vessels_.size()) {
        return core::fail(ErrorCode::OutOfRange, "unknown vessel id");
    }
    Vessel& vessel = vessels_[id.index];
    if (vessel.status == VesselStatus::Crashed || vessel.status == VesselStatus::Stowed
        || !vessel.systems.has_value()) {
        return core::fail(ErrorCode::InvalidArgument,
                          std::format("'{}' is not a vessel with systems to command", vessel.name));
    }
    return std::ref(*vessel.systems);
}

// After the vessel's systems changed at `instant`: the readings, the propagator's thrust plan
// and the predictions follow.
core::VoidResult Simulation::systems_changed(Vessel& vessel, const time::Epoch& instant) {
    if (!vessel.systems.has_value()) {
        return {};
    }
    if (core::VoidResult reported = vessel.systems->report_telemetry(instant); !reported) {
        return reported;
    }
    // Rationale: a landed vessel has no trajectory, and in the physics bubble the thrust is
    // applied tick by tick with the attitude the vessel really has, not planned ahead.
    const bool in_the_bubble =
        bubble_ != nullptr && bubble_->find(static_cast<std::size_t>(&vessel - vessels_.data())) != nullptr;
    if (vessel.status != VesselStatus::Flying || in_the_bubble) {
        return {};
    }
    if (core::VoidResult synced = sync_propulsion(vessel, instant, false); !synced) {
        return synced;
    }
    refresh_caches(vessel);
    return {};
}

core::VoidResult Simulation::command(VesselId id, std::string_view signal, vessel::ControlSource source,
                                     double value) {
    return change_command(id, signal, source, value);
}

core::VoidResult Simulation::release_command(VesselId id, std::string_view signal,
                                             vessel::ControlSource source) {
    return change_command(id, signal, source, std::nullopt);
}

core::VoidResult Simulation::change_command(VesselId id, std::string_view signal,
                                            vessel::ControlSource source, std::optional<double> value) {
    const auto systems = systems_of(id);
    if (!systems) {
        return std::unexpected(systems.error());
    }
    std::vector<vessel::Separation> separations;
    if (core::VoidResult changed = value.has_value()
                                       ? systems->get().command(signal, source, *value, now_, separations)
                                       : systems->get().release(signal, source, now_, separations);
        !changed) {
        return changed;
    }
    // Rationale: by index from here on, because a separation adds vessels and may move them.
    for (vessel::Separation& separation : separations) {
        if (core::VoidResult separated = vessels_[id.index].status == VesselStatus::Landed
                                             ? separate_landed(id.index, std::move(separation))
                                             : separate(id.index, std::move(separation));
            !separated) {
            return separated;
        }
    }
    return systems_changed(vessels_[id.index], now_);
}

core::VoidResult Simulation::schedule_command(VesselId id, vessel::TimedCommand command) {
    const auto systems = systems_of(id);
    if (!systems) {
        return std::unexpected(systems.error());
    }
    if (command.epoch < now_) {
        return core::fail(ErrorCode::OutOfRange, "a command cannot be scheduled in the past");
    }
    if (!systems->get().bus().find(command.signal).has_value()) {
        return core::fail(ErrorCode::InvalidArgument,
                          std::format("'{}' has no signal '{}'", vessels_[id.index].name, command.signal));
    }
    if (core::VoidResult scheduled = systems->get().schedule(std::move(command)); !scheduled) {
        return scheduled;
    }
    return systems_changed(vessels_[id.index], now_);
}

// Parts that left vessel `parent` become a vessel of their own, and the separator's push is
// applied to both as an impulse at the separation.
core::VoidResult Simulation::separate(std::size_t parent, vessel::Separation separation) {
    // A copy runs ahead to the separation; the parent's own propagator is untouched.
    dynamics::EnckePropagator ahead = vessels_[parent].propagator;
    const auto state = ahead.state_at(separation.epoch);
    if (!state) {
        return std::unexpected(state.error());
    }
    // Rationale: a vessel at rest about its body has no orbital frame to point in; the two
    // sides then part without a push rather than failing the separation.
    const auto direction =
        dynamics::thrust_unit_vector(to_thrust_direction(separation.pointing), state->state_in_domain);
    math::Vector3 along = direction ? *direction : math::Vector3{};
    // In the physics bubble the two sides are bodies of their own from here on (BRIEFING D30):
    // the push is along the nose as it really points, and each side's centre of mass is where
    // its parts are, moving as the turning vessel carried it.
    BubbleMember* const member = bubble_ != nullptr ? bubble_->find(parent) : nullptr;
    // Rationale: on rails the attitude is the commanded pointing at the separation itself, not
    // the one last shown, so that where the parts go does not depend on the frames (D14).
    const Attitude attitude = member != nullptr
                                  ? vessels_[parent].attitude
                                  : detail::pointing_attitude(separation.pointing, state->state_in_domain);
    math::Vector3 kept_shift_m;
    math::Vector3 separated_shift_m;
    if (member != nullptr) {
        along = math::rotate(attitude.orientation, {1.0, 0.0, 0.0});
        kept_shift_m = math::rotate(attitude.orientation, separation.kept_offset_m);
        separated_shift_m = math::rotate(attitude.orientation, separation.separated_offset_m);
        // Rationale: the propagator is behind the separation by a part of a tick and only moves
        // forward, so the kept side's new centre is given to it at the next tick.
        member->pending_shift_m += kept_shift_m;
        member->body_part_count = 0; // the rigid body no longer matches the vessel
    } else {
        // Rationale: the kept side's propagator is not disturbed for a few metres, so the parts
        // that leave take the whole distance between the two centres. That keeps the two where
        // they are relative to each other, which matters if they become bodies later.
        separated_shift_m =
            math::rotate(attitude.orientation, separation.separated_offset_m - separation.kept_offset_m);
    }
    const math::Vector3& spin_rad_s = attitude.angular_velocity_rad_s;
    if (core::VoidResult kicked = vessels_[parent].propagator.schedule_impulse(
            {.epoch = separation.epoch,
             .delta_v_m_s = separation.kept_delta_v_m_s * along + math::cross(spin_rad_s, kept_shift_m)});
        !kicked) {
        return kicked;
    }
    const dynamics::VesselState initial{
        .domain = state->domain,
        .epoch = separation.epoch,
        .state_in_domain = {.position_m = state->state_in_domain.position_m + separated_shift_m,
                            .velocity_m_s = state->state_in_domain.velocity_m_s
                                            + separation.separated_delta_v_m_s * along
                                            + math::cross(spin_rad_s, separated_shift_m)}};
    auto propagator = dynamics::EnckePropagator::make(*gravity_, initial, options_.propagator);
    if (!propagator) {
        return std::unexpected(propagator.error());
    }
    std::string name =
        std::format("{} ({})", vessels_[parent].name, separation.systems.assembly().parts().front().name);
    vessels_[parent].events_stale = true;
    vessels_[parent].prediction_stale = true;
    const bool in_the_bubble = member != nullptr;
    const math::Quaternion separated_orientation =
        math::normalized(attitude.orientation * separation.separated_orientation);
    vessels_.push_back(Vessel{.name = std::move(name),
                              .status = VesselStatus::Flying,
                              .propagator = std::move(*propagator),
                              .state = initial,
                              .systems = std::move(separation.systems),
                              .events = {},
                              .events_epoch = separation.epoch,
                              .prediction = {},
                              .prediction_start = separation.epoch,
                              .prediction_horizon_s = 0.0,
                              .events_stale = true,
                              .prediction_stale = true});
    Vessel& separated = vessels_.back();
    separated.attitude = {.orientation = separated_orientation,
                          .angular_velocity_rad_s = attitude.angular_velocity_rad_s};
    if (in_the_bubble) {
        if (core::VoidResult joined = join_bubble(vessels_.size() - 1, separation.epoch); !joined) {
            return joined;
        }
    }
    if (core::VoidResult changed = systems_changed(separated, separation.epoch); !changed) {
        return changed;
    }
    LOG_INFO("'{}' separated from '{}'", separated.name, vessels_[parent].name).tag("subsystem", "sim");
    return {};
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
        // An engine starting or stopping is a burn to stop the warp for, like an impulse.
        const auto thrust_changes = vessel.propagator.pending_thrust_changes();
        if (!thrust_changes.empty()) {
            consider(WarpLimit{.vessel = {index},
                               .epoch = thrust_changes.front().epoch,
                               .kind = {},
                               .body = vessel.state.domain});
        }
        for (const dynamics::PredictedEvent& event : vessel.events) {
            if (limits_warp(event.kind) && event.epoch >= now_) {
                consider(WarpLimit{
                    .vessel = {index}, .epoch = event.epoch, .kind = event.kind, .body = event.body});
                break;
            }
        }
        if (const auto pending = pending_domain_change({index})) {
            consider(*pending);
        }
    }
    return limit;
}

// Rationale: the event prediction reports nothing for a vessel that is already across a sphere
// of influence boundary, because the propagator changes its domain at its next step boundary.
// Without a limit there, a frame at high warp (months at 1e9×) would carry the vessel through
// the change and far into the new domain before any event of that domain could be predicted.
std::optional<WarpLimit> Simulation::pending_domain_change(VesselId id) const {
    const Vessel& vessel = vessels_[id.index];
    if (!options_.propagator.switch_domains) {
        return std::nullopt;
    }
    const auto domain = catalog_->body(vessel.state.domain);
    if (!domain) {
        return std::nullopt;
    }
    const bodies::Body& current = domain->get();
    const math::Vector3& position_m = vessel.state.state_in_domain.position_m;
    const double hysteresis = options_.propagator.domain_hysteresis;
    std::optional<dynamics::EventKind> kind;
    bodies::BodyId body = vessel.state.domain;
    if (current.domain_parent.has_value()
        && math::norm(position_m) > current.domain_radius_m * (1.0 + hysteresis)) {
        kind = dynamics::EventKind::DomainExit;
    } else {
        for (const bodies::BodyId child : catalog_->domain_children(vessel.state.domain)) {
            const bodies::Body& child_body = catalog_->body(child)->get();
            const auto offset = tree_->relative_state(child_body.frame, current.frame, now_);
            if (offset
                && math::norm(position_m - offset->position_m)
                       < child_body.domain_radius_m * (1.0 - hysteresis)) {
                kind = dynamics::EventKind::DomainEntry;
                body = child;
                break;
            }
        }
    }
    if (!kind.has_value()) {
        return std::nullopt;
    }
    // Before the propagator has stepped past `now_` its step end is not ahead of us yet: hold
    // the clock (real time) for a frame, which makes it take that step.
    const auto step_end = vessel.propagator.step_end();
    return WarpLimit{
        .vessel = id, .epoch = step_end ? std::max(*step_end, now_) : now_, .kind = kind, .body = body};
}

core::VoidResult Simulation::advance(double wall_dt_s) {
    const std::optional<WarpLimit> limit = warp_limit();
    const auto step = warp_.advance(
        now_, wall_dt_s, limit.has_value() ? std::optional<time::Epoch>(limit->epoch) : std::nullopt);
    if (!step) {
        return std::unexpected(step.error());
    }
    effective_warp_ = step->factor;
    // Rationale: events are only known up to the horizon of each vessel's last search. A frame
    // that went further could pass an encounter nobody has looked for yet, so it stops at that
    // frontier (where the search is repeated) without slowing the warp.
    time::Epoch target = step->epoch;
    for (const Vessel& vessel : vessels_) {
        if (vessel.status != VesselStatus::Flying) {
            continue;
        }
        const auto frontier = vessel.events_epoch.advanced_by(options_.event_horizon_s);
        if (frontier && *frontier > now_) {
            target = std::min(target, *frontier);
        }
    }
    if (core::VoidResult moved = advance_to(target); !moved) {
        return moved;
    }
    const bool reached_limit = limit.has_value() && target == limit->epoch;
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
    if (core::VoidResult updated = update_bubble_membership(); !updated) {
        return updated;
    }
    // The bubble first: a vessel that leaves it on the way (landed, or back on rails) is taken
    // the rest of the way below.
    if (bubble_ != nullptr) {
        if (core::VoidResult advanced = advance_in_bubble(instant); !advanced) {
            return advanced;
        }
    }
    // Rationale: by index, because a stage that separates during the frame is added to the
    // vessels (and is then advanced itself, further down the same loop).
    for (std::size_t index = 0; index < vessels_.size(); ++index) {
        if (bubble_ != nullptr && bubble_->find(index) != nullptr) {
            continue;
        }
        if (vessels_[index].status == VesselStatus::Landed) {
            if (core::VoidResult advanced = advance_landed(index, instant); !advanced) {
                return advanced;
            }
            continue;
        }
        if (vessels_[index].status != VesselStatus::Flying) {
            continue;
        }
        // The systems first: what changes in them during the frame is already in the
        // propagator's plan, except the parts that leave.
        bool systems_moved_on = false;
        std::vector<vessel::Separation> separations;
        if (std::optional<vessel::VesselSystems>& systems = vessels_[index].systems; systems.has_value()) {
            const time::Epoch latest_change = systems->propulsion().epoch;
            if (core::VoidResult advanced = systems->advance_to(instant, separations); !advanced) {
                return advanced;
            }
            systems_moved_on = systems->propulsion().epoch != latest_change;
        }
        for (vessel::Separation& separation : separations) {
            if (core::VoidResult separated = separate(index, std::move(separation)); !separated) {
                return separated;
            }
        }
        Vessel& vessel = vessels_[index];
        const bodies::BodyId domain_before = vessel.state.domain;
        const std::uint64_t impulses_before = vessel.propagator.statistics().impulses;
        const std::uint64_t thrust_changes_before = vessel.propagator.statistics().thrust_changes;
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
        if (vessel.systems.has_value()) {
            if (core::VoidResult reported = vessel.systems->report_telemetry(instant); !reported) {
                return reported;
            }
            if (systems_moved_on) {
                if (core::VoidResult synced = sync_propulsion(vessel, instant, false); !synced) {
                    return synced;
                }
            }
            vessel.attitude =
                detail::pointing_attitude(vessel.systems->propulsion().pointing, state->state_in_domain);
            // On rails only the thrust along the nose acts, and nothing else is felt aboard.
            const double mass_kg = vessel.systems->mass_kg(instant);
            const double thrust_n = vessel.systems->propulsion().thrust_n;
            vessel.proper_acceleration_m_s2 =
                thrust_n != 0.0 && mass_kg > 0.0
                    ? (thrust_n / mass_kg) * math::rotate(vessel.attitude.orientation, {1.0, 0.0, 0.0})
                    : math::Vector3{};
        }
        const bool changed = state->domain != domain_before
                             || vessel.propagator.statistics().impulses != impulses_before
                             || vessel.propagator.statistics().thrust_changes != thrust_changes_before;
        // Rationale: the events are those of the osculating conic, which a burn changes all the
        // time. The predicted path needs no such refresh: it is integrated with the burn in it.
        const bool conic_changing = vessel.propagator.is_thrusting();
        const bool event_passed = !vessel.events.empty() && vessel.events.front().epoch <= instant;
        const bool prediction_aging =
            time::seconds_between(vessel.prediction_start, instant) > 0.25 * vessel.prediction_horizon_s;
        // Rationale: events are searched up to a horizon, so one beyond it when they were last
        // predicted (an encounter years ahead) only appears if the search is repeated.
        const bool events_aging =
            time::seconds_between(vessel.events_epoch, instant) > 0.25 * options_.event_horizon_s;
        vessel.events_stale =
            vessel.events_stale || changed || conic_changing || event_passed || events_aging;
        vessel.prediction_stale = vessel.prediction_stale || changed || prediction_aging;
        refresh_caches(vessel);
    }
    now_ = instant;
    if (core::VoidResult moved = advance_pilot(instant); !moved) {
        return moved;
    }
    return report_navigation();
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
        vessel.events_epoch = vessel.state.epoch;
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
