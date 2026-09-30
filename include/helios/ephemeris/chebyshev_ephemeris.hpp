#ifndef HELIOS_EPHEMERIS_CHEBYSHEV_EPHEMERIS_HPP
#define HELIOS_EPHEMERIS_CHEBYSHEV_EPHEMERIS_HPP

#include "helios/core/error.hpp"
#include "helios/math/vector3.hpp"
#include "helios/orbital/kepler.hpp"
#include "helios/time/epoch.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace helios::ephemeris {

// Piecewise Chebyshev position series, one record per fixed-length time interval — the
// representation used by JPL's DE ephemerides (SPK type 2). Velocity and acceleration are
// the exact analytic derivatives of the position series, so the frame's indirect term
// (BRIEFING §4.3) is consistent with how the body actually moves.
class ChebyshevEphemeris {
public:
    // `coefficients_m` layout: [record][axis x,y,z][k], k < coefficients_per_axis.
    // Fails on non-positive/non-finite duration, zero counts, a size mismatch or non-finite data.
    [[nodiscard]] static core::Result<ChebyshevEphemeris> make(const time::Epoch& start,
                                                               double record_duration_s,
                                                               std::size_t coefficients_per_axis,
                                                               std::vector<double> coefficients_m) noexcept;

    [[nodiscard]] core::Result<orbital::StateVector> state_at(const time::Epoch& instant) const noexcept;
    [[nodiscard]] core::Result<math::Vector3> acceleration_at(const time::Epoch& instant) const noexcept;

    // [start, start + record_count · duration]; the end instant itself is accepted.
    [[nodiscard]] std::optional<time::Interval> valid_range() const noexcept { return range_; }

    [[nodiscard]] std::size_t record_count() const noexcept { return record_count_; }
    [[nodiscard]] std::size_t coefficients_per_axis() const noexcept { return coefficients_per_axis_; }
    [[nodiscard]] double record_duration_s() const noexcept { return record_duration_s_; }

private:
    struct Derivatives {
        math::Vector3 position_m;
        math::Vector3 velocity_m_s;
        math::Vector3 acceleration_m_s2;
    };

    ChebyshevEphemeris(const time::Epoch& start, const time::Interval& range, double record_duration_s,
                       std::size_t record_count, std::size_t coefficients_per_axis,
                       std::vector<double> coefficients_m) noexcept;

    [[nodiscard]] core::Result<Derivatives> evaluate(const time::Epoch& instant) const noexcept;

    time::Epoch start_;
    time::Interval range_;
    double record_duration_s_;
    std::size_t record_count_;
    std::size_t coefficients_per_axis_;
    std::vector<double> coefficients_m_;
};

} // namespace helios::ephemeris

#endif // HELIOS_EPHEMERIS_CHEBYSHEV_EPHEMERIS_HPP
