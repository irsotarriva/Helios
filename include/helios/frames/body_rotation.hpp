#ifndef HELIOS_FRAMES_BODY_ROTATION_HPP
#define HELIOS_FRAMES_BODY_ROTATION_HPP

#include "helios/core/error.hpp"
#include "helios/math/matrix3.hpp"
#include "helios/math/vector3.hpp"
#include "helios/orbital/kepler.hpp"
#include "helios/time/epoch.hpp"

namespace helios::frames {

// IAU WGCCRE-style rotation model (linear terms): the north pole's right ascension and
// declination drift linearly per Julian century, the prime meridian angle W grows linearly per
// day. Angles are given in the *reference* equatorial frame (ICRF) and converted to the
// universe's axes (J2000 ecliptic for the stock Solar System) with `reference_to_universe`.
// Closed form in time, like the ephemerides: O(1) at any epoch, any warp.
struct RotationElements {
    double pole_right_ascension_rad = 0.0;
    double pole_right_ascension_rate_rad_s = 0.0;
    double pole_declination_rad = k_half_pi;
    double pole_declination_rate_rad_s = 0.0;
    double prime_meridian_rad = 0.0;  // W at the reference epoch
    double rotation_rate_rad_s = 0.0; // dW/dt; negative for retrograde rotators (Venus, Uranus)

    static constexpr double k_half_pi = 1.57079632679489661923;
};

// The body-fixed (rotating) frame of one body. Its origin is the body's inertial frame origin;
// only the axes differ.
class BodyRotation {
public:
    // `reference_to_universe`: rotation from the axes the elements are given in (ICRF) to the
    // universe axes. Fails on non-finite elements.
    [[nodiscard]] static core::Result<BodyRotation> make(const RotationElements& elements,
                                                         const math::Matrix3& reference_to_universe) noexcept;

    // Rotation taking universe-axis coordinates to body-fixed coordinates at `instant`.
    [[nodiscard]] math::Matrix3 universe_to_body_fixed(const time::Epoch& instant) const noexcept;

    // Body angular velocity ω, in universe axes. The slow pole drift (~1e-12 rad/s) is neglected.
    [[nodiscard]] math::Vector3 angular_velocity_rad_s(const time::Epoch& instant) const noexcept;

    // Position/velocity relative to the body centre: inertial (universe axes) ↔ body-fixed.
    // v_fixed = R (v − ω × r) and its inverse.
    [[nodiscard]] orbital::StateVector to_body_fixed(const orbital::StateVector& inertial,
                                                     const time::Epoch& instant) const noexcept;
    [[nodiscard]] orbital::StateVector from_body_fixed(const orbital::StateVector& body_fixed,
                                                       const time::Epoch& instant) const noexcept;

    // Sidereal rotation period (2π/|dW/dt|).
    [[nodiscard]] double sidereal_period_s() const noexcept;

private:
    BodyRotation(const RotationElements& elements, const math::Matrix3& reference_to_universe) noexcept
        : elements_(elements), reference_to_universe_(reference_to_universe) {}

    RotationElements elements_;
    math::Matrix3 reference_to_universe_;
};

// ICRF → J2000 ecliptic (obliquity 84381.448″), the stock universe's axes.
[[nodiscard]] math::Matrix3 icrf_to_j2000_ecliptic() noexcept;

// Planetocentric latitude/longitude/radius of a body-fixed position (longitude east-positive).
struct SphericalPosition {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double radius_m = 0.0;
};
[[nodiscard]] SphericalPosition to_spherical(const math::Vector3& body_fixed_position_m) noexcept;

} // namespace helios::frames

#endif // HELIOS_FRAMES_BODY_ROTATION_HPP
