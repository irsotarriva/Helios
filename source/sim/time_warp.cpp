#include "helios/sim/time_warp.hpp"

#include <algorithm>
#include <cmath>

namespace helios::sim {

void TimeWarp::set_max_level(std::size_t level) noexcept {
    max_level_ = std::min(level, k_warp_levels.size() - 1);
    requested_level_ = std::min(requested_level_, max_level_);
}

void TimeWarp::request_level(std::size_t level) noexcept {
    requested_level_ = std::min(level, max_level_);
}

double TimeWarp::limited_factor(std::optional<double> time_to_event_s) const noexcept {
    const double requested = requested_factor();
    if (!time_to_event_s.has_value() || requested <= 1.0) {
        return requested;
    }
    return std::clamp(*time_to_event_s / k_event_lead_wall_s, 1.0, requested);
}

core::Result<TimeWarp::Advance> TimeWarp::advance(const time::Epoch& now, double wall_dt_s,
                                                  std::optional<time::Epoch> next_event) const noexcept {
    if (!(wall_dt_s >= 0.0) || !std::isfinite(wall_dt_s)) {
        return core::fail(core::ErrorCode::OutOfRange,
                          "wall-clock frame time must be finite and non-negative");
    }
    if (paused_) {
        return Advance{.epoch = now, .factor = 0.0};
    }
    std::optional<double> time_to_event_s;
    if (next_event.has_value() && *next_event >= now) {
        time_to_event_s = time::seconds_between(now, *next_event);
    }
    const double factor = limited_factor(time_to_event_s);
    const double step_s = factor * wall_dt_s;
    if (next_event.has_value() && time_to_event_s.has_value() && *time_to_event_s > 0.0
        && step_s >= *time_to_event_s) {
        return Advance{.epoch = *next_event, .factor = factor};
    }
    return now.advanced_by(step_s).transform(
        [&](const time::Epoch& epoch) { return Advance{.epoch = epoch, .factor = factor}; });
}

} // namespace helios::sim
