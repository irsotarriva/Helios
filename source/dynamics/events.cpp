#include "helios/dynamics/events.hpp"

#include "helios/orbital/conic.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace helios::dynamics {

namespace {

using core::ErrorCode;

constexpr int k_max_entry_steps = 4096;
constexpr int k_bisection_iterations = 48;
constexpr int k_max_apsis_passages = 32;

struct Separation {
    double distance_m = 0.0;
    double relative_speed_m_s = 0.0;
};

// Vessel (on its conic) relative to a child body, `elapsed_s` after the vessel's epoch.
[[nodiscard]] core::Result<Separation> separation_at(const frames::FrameTree& tree,
                                                     const bodies::Body& domain, const bodies::Body& child,
                                                     const VesselState& vessel, double mu_m3_s2,
                                                     double elapsed_s) noexcept {
    const auto vessel_state = orbital::propagate_conic(vessel.state_in_domain, mu_m3_s2, elapsed_s);
    if (!vessel_state) {
        return std::unexpected(vessel_state.error());
    }
    const auto instant = vessel.epoch.advanced_by(elapsed_s);
    if (!instant) {
        return std::unexpected(instant.error());
    }
    return tree.relative_state(child.frame, domain.frame, *instant)
        .transform([&](const orbital::StateVector& child_state) {
            return Separation{.distance_m = math::norm(vessel_state->position_m - child_state.position_m),
                              .relative_speed_m_s =
                                  math::norm(vessel_state->velocity_m_s - child_state.velocity_m_s)};
        });
}

// First time in (0, horizon] at which the vessel's conic is inside `radius_m` of the child.
[[nodiscard]] core::Result<std::optional<double>>
first_entry_s(const frames::FrameTree& tree, const bodies::Body& domain, const bodies::Body& child,
              const VesselState& vessel, double mu_m3_s2, double radius_m, double horizon_s) noexcept {
    const double min_step_s = horizon_s / static_cast<double>(k_max_entry_steps);
    double previous_s = 0.0;
    auto previous = separation_at(tree, domain, child, vessel, mu_m3_s2, 0.0);
    if (!previous) {
        return std::unexpected(previous.error());
    }
    if (previous->distance_m < radius_m) {
        return std::optional<double>{}; // already inside: the propagator switches at its next step
    }
    for (int step = 0; step < k_max_entry_steps && previous_s < horizon_s; ++step) {
        const double gap_m = previous->distance_m - radius_m;
        const double step_s =
            std::max(0.5 * gap_m / std::max(previous->relative_speed_m_s, 1e-3), min_step_s);
        const double time_s = std::min(previous_s + step_s, horizon_s);
        const auto current = separation_at(tree, domain, child, vessel, mu_m3_s2, time_s);
        if (!current) {
            return std::unexpected(current.error());
        }
        if (current->distance_m < radius_m) {
            double outside_s = previous_s;
            double inside_s = time_s;
            for (int iteration = 0; iteration < k_bisection_iterations; ++iteration) {
                const double middle_s = 0.5 * (outside_s + inside_s);
                const auto middle = separation_at(tree, domain, child, vessel, mu_m3_s2, middle_s);
                if (!middle) {
                    return std::unexpected(middle.error());
                }
                (middle->distance_m < radius_m ? inside_s : outside_s) = middle_s;
            }
            return std::optional<double>{inside_s};
        }
        previous_s = time_s;
        previous = current;
    }
    return std::optional<double>{};
}

} // namespace

std::string_view to_string(EventKind kind) noexcept {
    switch (kind) {
    case EventKind::Periapsis:     return "periapsis";
    case EventKind::Apoapsis:      return "apoapsis";
    case EventKind::DomainExit:    return "sphere-of-influence exit";
    case EventKind::DomainEntry:   return "sphere-of-influence entry";
    case EventKind::SurfaceImpact: return "surface impact";
    }
    return "unknown";
}

core::Result<std::vector<PredictedEvent>> predict_conic_events(const GravityModel& gravity,
                                                               const VesselState& vessel,
                                                               const EventSearchOptions& options) {
    if (!(options.horizon_s > 0.0) || !std::isfinite(options.horizon_s) || options.domain_hysteresis < 0.0
        || options.domain_hysteresis >= 1.0) {
        return core::fail(ErrorCode::OutOfRange, "invalid event search options");
    }
    const bodies::BodyCatalog& catalog = gravity.catalog();
    const auto domain_body = catalog.body(vessel.domain);
    if (!domain_body) {
        return std::unexpected(domain_body.error());
    }
    const bodies::Body& domain = domain_body->get();
    const double mu_m3_s2 = domain.gravitational_parameter_m3_s2;
    const auto conic = orbital::conic_geometry(vessel.state_in_domain, mu_m3_s2);
    if (!conic) {
        return std::unexpected(conic.error());
    }

    struct Candidate {
        EventKind kind;
        double time_s;
        bodies::BodyId body;
    };
    std::vector<Candidate> candidates;
    const auto consider = [&](EventKind kind, std::optional<double> time_s, bodies::BodyId body) {
        if (time_s.has_value() && *time_s <= options.horizon_s) {
            candidates.push_back({kind, *time_s, body});
        }
    };

    // Terminal events: the conic stops describing the trajectory after the first of these.
    consider(EventKind::SurfaceImpact,
             orbital::time_to_radius_s(*conic, domain.mean_radius_m, orbital::RadialDirection::Inbound),
             vessel.domain);
    if (domain.domain_parent.has_value()) {
        consider(EventKind::DomainExit,
                 orbital::time_to_radius_s(*conic, domain.domain_radius_m * (1.0 + options.domain_hysteresis),
                                           orbital::RadialDirection::Outbound),
                 vessel.domain);
    }
    for (const bodies::BodyId child_id : catalog.domain_children(vessel.domain)) {
        const bodies::Body& child = catalog.body(child_id)->get();
        const double entry_radius_m = child.domain_radius_m * (1.0 - options.domain_hysteresis);
        // Rationale: a conic that never comes within reach of the child's orbit cannot enter its
        // sphere; skip the stepping search (the common case for most planets from a given orbit).
        const auto child_now = gravity.tree().relative_state(child.frame, domain.frame, vessel.epoch);
        if (!child_now) {
            return std::unexpected(child_now.error());
        }
        const double child_radius_m = math::norm(child_now->position_m);
        if (conic->apoapsis_radius_m < 0.5 * child_radius_m - entry_radius_m
            || conic->periapsis_radius_m > 2.0 * child_radius_m + entry_radius_m) {
            continue;
        }
        double search_end_s = options.horizon_s;
        for (const Candidate& candidate : candidates) {
            search_end_s = std::min(search_end_s, candidate.time_s);
        }
        const auto entry =
            first_entry_s(gravity.tree(), domain, child, vessel, mu_m3_s2, entry_radius_m, search_end_s);
        if (!entry) {
            return std::unexpected(entry.error());
        }
        consider(EventKind::DomainEntry, *entry, child_id);
    }
    double terminal_s = options.horizon_s;
    for (const Candidate& candidate : candidates) {
        terminal_s = std::min(terminal_s, candidate.time_s);
    }

    // Apsides repeat every period; report each passage before the terminal event.
    const double period_s = orbital::orbital_period_s(*conic);
    const auto add_repeating = [&](EventKind kind, std::optional<double> first_s) {
        if (!first_s.has_value()) {
            return;
        }
        for (int passage = 0; passage < k_max_apsis_passages; ++passage) {
            const double time_s = *first_s + static_cast<double>(passage) * period_s;
            if (time_s > terminal_s) {
                break;
            }
            candidates.push_back({kind, time_s, vessel.domain});
            if (!std::isfinite(period_s)) {
                break;
            }
        }
    };
    add_repeating(EventKind::Periapsis, orbital::time_to_periapsis_s(*conic));
    add_repeating(EventKind::Apoapsis, orbital::time_to_apoapsis_s(*conic));

    std::ranges::sort(candidates, {}, &Candidate::time_s);
    std::vector<PredictedEvent> events;
    for (const Candidate& candidate : candidates) {
        if (candidate.time_s > terminal_s) {
            break;
        }
        const auto epoch = vessel.epoch.advanced_by(candidate.time_s);
        if (!epoch) {
            return std::unexpected(epoch.error());
        }
        events.push_back({candidate.kind, *epoch, candidate.body});
        if (candidate.kind == EventKind::SurfaceImpact || candidate.kind == EventKind::DomainExit
            || candidate.kind == EventKind::DomainEntry) {
            break;
        }
    }
    return events;
}

} // namespace helios::dynamics
