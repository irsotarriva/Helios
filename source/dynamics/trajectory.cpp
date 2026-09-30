#include "helios/dynamics/trajectory.hpp"

#include <algorithm>
#include <cmath>

namespace helios::dynamics {

core::Result<std::vector<TrajectorySegment>>
predict_trajectory(EnckePropagator propagator, const time::Epoch& from, const PredictionOptions& options) {
    if (!(options.horizon_s > 0.0) || !std::isfinite(options.horizon_s)
        || !(options.max_true_anomaly_step_rad > 0.0) || !(options.min_step_s > 0.0)
        || options.max_samples < 2) {
        return core::fail(core::ErrorCode::OutOfRange, "invalid trajectory prediction options");
    }
    const bodies::BodyCatalog& catalog = propagator.gravity().catalog();
    const double max_step_s = std::max(options.min_step_s, options.horizon_s / 8.0);
    std::vector<TrajectorySegment> segments;
    std::size_t sample_count = 0;
    double elapsed_s = 0.0;
    for (;;) {
        const auto instant = from.advanced_by(elapsed_s);
        if (!instant) {
            return std::unexpected(instant.error());
        }
        const auto state = propagator.state_at(*instant);
        if (!state) {
            if (sample_count > 0) {
                break;
            }
            return std::unexpected(state.error());
        }
        if (segments.empty() || segments.back().domain != state->domain) {
            segments.push_back(TrajectorySegment{.domain = state->domain, .samples = {}});
        }
        segments.back().samples.push_back({*instant, state->state_in_domain.position_m});
        ++sample_count;

        const double radius_m = math::norm(state->state_in_domain.position_m);
        const auto domain = catalog.body(state->domain);
        if (!domain) {
            return std::unexpected(domain.error());
        }
        if (radius_m <= domain->get().mean_radius_m || elapsed_s >= options.horizon_s
            || sample_count >= options.max_samples) {
            break;
        }
        const double angular_momentum_m2_s =
            math::norm(math::cross(state->state_in_domain.position_m, state->state_in_domain.velocity_m_s));
        const double step_s = angular_momentum_m2_s > 0.0 ? options.max_true_anomaly_step_rad * radius_m
                                                                * radius_m / angular_momentum_m2_s
                                                          : max_step_s;
        elapsed_s =
            std::min(elapsed_s + std::clamp(step_s, options.min_step_s, max_step_s), options.horizon_s);
    }
    return segments;
}

} // namespace helios::dynamics
