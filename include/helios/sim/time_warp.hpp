#ifndef HELIOS_SIM_TIME_WARP_HPP
#define HELIOS_SIM_TIME_WARP_HPP

#include "helios/core/error.hpp"
#include "helios/time/epoch.hpp"

#include <array>
#include <cstddef>
#include <optional>

namespace helios::sim {

// Warp factors, simulated seconds per wall-clock second (BRIEFING §7: 0.01× to 1e6×).
inline constexpr std::array<double, 9> k_warp_levels{0.01, 0.1, 1.0, 10.0, 100.0, 1e3, 1e4, 1e5, 1e6};
inline constexpr std::size_t k_real_time_level = 2;

// The time-warp controller. It only decides how far the universe clock moves per frame; the
// propagators are warp-invariant (D14), so warp never changes a trajectory, only how much of it
// is computed per frame.
class TimeWarp {
public:
    // The event lead: warp is reduced so the next limiting event stays at least this much wall
    // time away. Because the limit is proportional to the time left, the approach is an
    // exponential ramp: from 1e6× to real time takes lead · ln(1e6) ≈ 7 s of wall time.
    static constexpr double k_event_lead_wall_s = 0.5;

    struct Advance {
        time::Epoch epoch;
        double factor = 1.0; // the warp actually used this frame
    };

    [[nodiscard]] std::size_t requested_level() const noexcept { return requested_level_; }
    [[nodiscard]] double requested_factor() const noexcept { return k_warp_levels.at(requested_level_); }
    [[nodiscard]] bool paused() const noexcept { return paused_; }

    // Out-of-range levels are clamped.
    void request_level(std::size_t level) noexcept;
    void increase() noexcept { request_level(requested_level_ + 1); }
    void decrease() noexcept {
        if (requested_level_ > 0) {
            request_level(requested_level_ - 1);
        }
    }
    void set_paused(bool paused) noexcept { paused_ = paused; }

    // The requested factor, reduced (continuously, not to a level) so `time_to_event_s` stays at
    // least k_event_lead_wall_s of wall time away; never below real time unless real time or
    // slower was requested. Without an event it is the requested factor.
    [[nodiscard]] double limited_factor(std::optional<double> time_to_event_s) const noexcept;

    // Moves `now` by wall_dt · limited factor, never past a strictly future `next_event`
    // (landing exactly on it). A `next_event` in the past is ignored. Fails on a negative or
    // non-finite wall_dt.
    [[nodiscard]] core::Result<Advance> advance(const time::Epoch& now, double wall_dt_s,
                                                std::optional<time::Epoch> next_event) const noexcept;

private:
    std::size_t requested_level_ = k_real_time_level;
    bool paused_ = false;
};

} // namespace helios::sim

#endif // HELIOS_SIM_TIME_WARP_HPP
