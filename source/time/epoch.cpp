#include "helios/time/epoch.hpp"

#include <cmath>
#include <limits>

namespace helios::time {

namespace {

using core::ErrorCode;

// 2^63 as a double; exactly representable. Valid int64 doubles lie in [-2^63, 2^63).
constexpr double k_int64_bound = 9'223'372'036'854'775'808.0;

[[nodiscard]] bool add_overflows(std::int64_t lhs, std::int64_t rhs) noexcept {
    constexpr std::int64_t k_max = std::numeric_limits<std::int64_t>::max();
    constexpr std::int64_t k_min = std::numeric_limits<std::int64_t>::min();
    return (rhs > 0 && lhs > k_max - rhs) || (rhs < 0 && lhs < k_min - rhs);
}

[[nodiscard]] bool subtract_overflows(std::int64_t lhs, std::int64_t rhs) noexcept {
    constexpr std::int64_t k_max = std::numeric_limits<std::int64_t>::max();
    constexpr std::int64_t k_min = std::numeric_limits<std::int64_t>::min();
    return (rhs < 0 && lhs > k_max + rhs) || (rhs > 0 && lhs < k_min + rhs);
}

struct SplitSeconds {
    std::int64_t whole;
    double fraction; // in [0, 1)
};

// Splits a finite double into an exact integer part and a fraction in [0, 1).
[[nodiscard]] core::Result<SplitSeconds> split(double seconds) noexcept {
    if (!std::isfinite(seconds)) {
        return core::fail(ErrorCode::NotFinite, "time value is NaN or infinite");
    }
    double whole = std::floor(seconds);
    // Exact for seconds >= 0 and seconds <= -1 (Sterbenz lemma). In (-1, 0) the sum can round:
    // e.g. -1e-20 gives floor = -1 and a fraction that rounds up to exactly 1.0.
    double fraction = seconds - whole;
    if (fraction >= 1.0) {
        fraction = 0.0;
        whole += 1.0;
    }
    if (whole < -k_int64_bound || whole >= k_int64_bound) {
        return core::fail(ErrorCode::Overflow, "time value exceeds the int64 range of whole seconds");
    }
    return SplitSeconds{.whole = static_cast<std::int64_t>(whole), .fraction = fraction};
}

} // namespace

core::Result<Epoch> Epoch::from_parts(std::int64_t whole_seconds, double fractional_seconds) noexcept {
    return split(fractional_seconds).and_then([&](const SplitSeconds parts) -> core::Result<Epoch> {
        if (add_overflows(whole_seconds, parts.whole)) {
            return core::fail(ErrorCode::Overflow, "epoch whole seconds overflow int64");
        }
        return Epoch{whole_seconds + parts.whole, parts.fraction};
    });
}

core::Result<Epoch> Epoch::from_seconds(double seconds_since_reference) noexcept {
    return from_parts(0, seconds_since_reference);
}

double Epoch::to_seconds() const noexcept {
    return static_cast<double>(whole_seconds__) + fraction__;
}

core::Result<Epoch> Epoch::advanced_by(double offset_seconds) const noexcept {
    return split(offset_seconds).and_then([&](const SplitSeconds offset) -> core::Result<Epoch> {
        if (add_overflows(whole_seconds__, offset.whole)) {
            return core::fail(ErrorCode::Overflow, "advancing epoch overflows int64 whole seconds");
        }
        // Both fractions are in [0, 1), so the sum is in [0, 2) and from_parts carries at most 1.
        return from_parts(whole_seconds__ + offset.whole, fraction__ + offset.fraction);
    });
}

double seconds_between(const Epoch& from, const Epoch& to) noexcept {
    const double fraction_difference = to.fraction() - from.fraction();
    if (subtract_overflows(to.whole_seconds(), from.whole_seconds())) [[unlikely]] {
        // Only reachable for epochs ~1e11 years apart; double precision is all we can offer then.
        return (static_cast<double>(to.whole_seconds()) - static_cast<double>(from.whole_seconds()))
               + fraction_difference;
    }
    return static_cast<double>(to.whole_seconds() - from.whole_seconds()) + fraction_difference;
}

core::Result<Interval> Interval::make(const Epoch& begin, const Epoch& end) noexcept {
    if (end < begin) {
        return core::fail(ErrorCode::InvalidArgument, "interval end precedes its begin");
    }
    return Interval{begin, end};
}

bool Interval::contains(const Epoch& instant) const noexcept {
    return begin__ <= instant && instant < end__;
}

double Interval::duration_seconds() const noexcept {
    return seconds_between(begin__, end__);
}

} // namespace helios::time
