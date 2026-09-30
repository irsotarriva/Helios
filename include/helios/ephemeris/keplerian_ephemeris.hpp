#ifndef HELIOS_EPHEMERIS_KEPLERIAN_EPHEMERIS_HPP
#define HELIOS_EPHEMERIS_KEPLERIAN_EPHEMERIS_HPP

#include "helios/core/error.hpp"
#include "helios/math/vector3.hpp"
#include "helios/orbital/kepler.hpp"
#include "helios/time/epoch.hpp"

#include <optional>

namespace helios::ephemeris {

// Mean elements at a reference epoch plus linear (secular) rates, in the style of
// Standish's "Keplerian elements for approximate positions of the major planets".
// Angles use longitudes, which stay well defined for low inclinations/eccentricities.
struct SecularElements {
    time::Epoch reference_epoch;
    double semi_major_axis_m = 0.0;
    double semi_major_axis_rate_m_s = 0.0;
    double eccentricity = 0.0;
    double eccentricity_rate_per_s = 0.0;
    double inclination_rad = 0.0;
    double inclination_rate_rad_s = 0.0;
    double longitude_of_ascending_node_rad = 0.0;
    double longitude_of_ascending_node_rate_rad_s = 0.0;
    double longitude_of_periapsis_rad = 0.0; // ϖ = Ω + ω
    double longitude_of_periapsis_rate_rad_s = 0.0;
    double mean_longitude_rad = 0.0;        // L = ϖ + M
    double mean_longitude_rate_rad_s = 0.0; // the mean motion n
};

// Closed-form body motion relative to its parent frame: O(1) at any epoch, no integration.
//
// Rationale: the velocity and acceleration are those of the instantaneous Kepler orbit with
// an effective μ = n²a³ (n = mean-longitude rate). This makes position, velocity and
// acceleration mutually consistent up to the tiny secular drift of the other elements
// (relative size ~ |ϖ̇|/n ≲ 1e-5 for the planets), which BRIEFING §4.3 requires for the
// frame's indirect acceleration term.
class KeplerianEphemeris {
public:
    // Fails on non-finite input, a ≤ 0, e ∉ [0, 1) or n ≤ 0.
    [[nodiscard]] static core::Result<KeplerianEphemeris> make(const SecularElements& elements) noexcept;

    [[nodiscard]] const SecularElements& elements() const noexcept { return elements_; }

    // Osculating elements at `instant`. Fails if the rates drive a or e out of range
    // (i.e. `instant` is far outside the fit window).
    [[nodiscard]] core::Result<orbital::KeplerianElements>
    elements_at(const time::Epoch& instant) const noexcept;

    [[nodiscard]] core::Result<orbital::StateVector> state_at(const time::Epoch& instant) const noexcept;
    [[nodiscard]] core::Result<math::Vector3> acceleration_at(const time::Epoch& instant) const noexcept;

    // Unbounded: secular models evaluate anywhere (accuracy degrades away from the fit window).
    // NOLINTNEXTLINE(readability-convert-member-functions-to-static): part of the EphemerisModel concept.
    [[nodiscard]] std::optional<time::Interval> valid_range() const noexcept { return std::nullopt; }

    // μ_eff = n²a³ at `instant`.
    [[nodiscard]] double effective_gravitational_parameter_m3_s2(double semi_major_axis_m) const noexcept;

private:
    explicit KeplerianEphemeris(const SecularElements& elements) noexcept : elements_(elements) {}

    SecularElements elements_;
};

} // namespace helios::ephemeris

#endif // HELIOS_EPHEMERIS_KEPLERIAN_EPHEMERIS_HPP
