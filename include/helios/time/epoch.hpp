#ifndef HELIOS_TIME_EPOCH_HPP
#define HELIOS_TIME_EPOCH_HPP

#include "helios/core/error.hpp"

#include <compare>
#include <cstdint>

namespace helios::time {

inline constexpr double k_seconds_per_day = 86'400.0;
inline constexpr double k_seconds_per_julian_year = 31'557'600.0;

// An instant of universe coordinate time, measured from the universe's reference epoch
// (J2000 TDB, barycentric, for the stock Solar System; see BRIEFING §6.4).
//
// Rationale: a plain double of seconds degrades to ~4 s resolution after 1e9 years
// (BRIEFING §3.2). Splitting into exact whole seconds (±2.9e11 years) plus a fraction in
// [0, 1) keeps ~1e-16 s resolution at any epoch. Integrators never see an Epoch; they
// work in local double offsets obtained from seconds_between().
class Epoch {
public:
    // The reference epoch itself.
    constexpr Epoch() noexcept = default;

    // Normalises any finite `fractional_seconds` (any sign or magnitude) into [0, 1),
    // carrying into the whole seconds. Fails on NaN/inf or int64 overflow.
    [[nodiscard]] static core::Result<Epoch> from_parts(std::int64_t whole_seconds,
                                                        double fractional_seconds) noexcept;

    // Convenience for small, human-scale values; precision is limited by the double.
    [[nodiscard]] static core::Result<Epoch> from_seconds(double seconds_since_reference) noexcept;

    [[nodiscard]] constexpr std::int64_t whole_seconds() const noexcept { return whole_seconds__; }

    // Always in [0, 1).
    [[nodiscard]] constexpr double fraction() const noexcept { return fraction__; }

    // Lossy: resolution degrades with distance from the reference epoch.
    [[nodiscard]] double to_seconds() const noexcept;

    // This instant moved by `offset_seconds` (may be negative). The fractional part of
    // the offset is preserved exactly, whatever the magnitude of the epoch.
    [[nodiscard]] core::Result<Epoch> advanced_by(double offset_seconds) const noexcept;

    // Lexicographic comparison is correct because the fraction is normalised.
    [[nodiscard]] friend constexpr bool operator==(const Epoch&, const Epoch&) noexcept = default;
    [[nodiscard]] friend constexpr std::partial_ordering operator<=>(const Epoch&,
                                                                     const Epoch&) noexcept = default;

private:
    constexpr Epoch(std::int64_t whole_seconds, double fraction) noexcept
        : whole_seconds__(whole_seconds), fraction__(fraction) {}

    std::int64_t whole_seconds__ = 0;
    double fraction__ = 0.0;
};

// `to - from` in seconds. The whole-second difference is exact before conversion, so the
// result is accurate to one ULP of the *difference*, not of the epochs.
[[nodiscard]] double seconds_between(const Epoch& from, const Epoch& to) noexcept;

// Half-open span [begin, end) of coordinate time, as used by worldline segments.
class Interval {
public:
    // Fails if `end` precedes `begin`. An empty interval (begin == end) is allowed.
    [[nodiscard]] static core::Result<Interval> make(const Epoch& begin, const Epoch& end) noexcept;

    [[nodiscard]] constexpr const Epoch& begin() const noexcept { return begin__; }
    [[nodiscard]] constexpr const Epoch& end() const noexcept { return end__; }

    [[nodiscard]] bool contains(const Epoch& instant) const noexcept;
    [[nodiscard]] double duration_seconds() const noexcept;

private:
    constexpr Interval(const Epoch& begin, const Epoch& end) noexcept : begin__(begin), end__(end) {}

    Epoch begin__;
    Epoch end__;
};

} // namespace helios::time

#endif // HELIOS_TIME_EPOCH_HPP
