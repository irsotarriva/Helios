#include "helios/frames/body_rotation.hpp"
#include "helios/orbital/kepler.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace {

using helios::frames::BodyRotation;
using helios::frames::icrf_to_j2000_ecliptic;
using helios::frames::RotationElements;
using helios::frames::to_spherical;
using helios::math::norm;
using helios::math::Vector3;
using helios::orbital::StateVector;
using helios::time::Epoch;

constexpr double k_degree_rad = std::numbers::pi / 180.0;
constexpr double k_earth_mu_m3_s2 = 3.986004354e14;

// IAU Earth: α₀ = 0 − 0.641T, δ₀ = 90 − 0.557T, W = 190.147 + 360.9856235 d.
[[nodiscard]] BodyRotation earth_rotation() {
    constexpr double k_century_s = 36'525.0 * 86'400.0;
    return BodyRotation::make(
               RotationElements{.pole_right_ascension_rad = 0.0,
                                .pole_right_ascension_rate_rad_s = -0.641 * k_degree_rad / k_century_s,
                                .pole_declination_rad = 90.0 * k_degree_rad,
                                .pole_declination_rate_rad_s = -0.557 * k_degree_rad / k_century_s,
                                .prime_meridian_rad = 190.147 * k_degree_rad,
                                .rotation_rate_rad_s = 360.9856235 * k_degree_rad / 86'400.0},
               icrf_to_j2000_ecliptic())
        .value();
}

TEST(BodyRotation, EarthHasASiderealDayAndItsObliquity) {
    const BodyRotation earth = earth_rotation();
    // The IAU rate is relative to the ICRF (a stellar day), 86164.0962 s, not the
    // precession-relative sidereal day of 86164.0905 s.
    EXPECT_NEAR(earth.sidereal_period_s(), 86'164.0962, 1e-3);
    // In ecliptic axes the spin axis is tilted by the obliquity, 23.439°.
    const Vector3 spin_axis = earth.angular_velocity_rad_s(Epoch{});
    EXPECT_NEAR(std::acos(spin_axis.z / norm(spin_axis)) / k_degree_rad, 23.4393, 1e-3);
}

TEST(BodyRotation, TransformsRoundTrip) {
    const BodyRotation earth = earth_rotation();
    const Epoch instant = Epoch::from_seconds(1.234e8).value();
    const StateVector inertial{.position_m = {7.0e6, -1.0e6, 2.0e6},
                               .velocity_m_s = {100.0, 7'400.0, -300.0}};
    const StateVector back = earth.from_body_fixed(earth.to_body_fixed(inertial, instant), instant);
    EXPECT_LT(norm(back.position_m - inertial.position_m), 1e-8);
    EXPECT_LT(norm(back.velocity_m_s - inertial.velocity_m_s), 1e-11);
}

TEST(BodyRotation, GeostationaryOrbitHoldsItsLongitude) {
    const BodyRotation earth = earth_rotation();
    const double rate_rad_s = helios::orbital::k_two_pi / earth.sidereal_period_s();
    const double radius_m = std::cbrt(k_earth_mu_m3_s2 / (rate_rad_s * rate_rad_s));

    // A body-fixed point on the equator, at rest in the rotating frame, is a circular equatorial
    // orbit in inertial space if its radius is the geostationary one.
    const StateVector fixed_point{.position_m = {radius_m, 0.0, 0.0}, .velocity_m_s = {}};
    const StateVector inertial = earth.from_body_fixed(fixed_point, Epoch{});
    EXPECT_NEAR(norm(inertial.velocity_m_s), rate_rad_s * radius_m, 1e-6);

    for (const double days : {1.0, 10.0, 30.0}) {
        const Epoch later = Epoch::from_seconds(days * 86'400.0).value();
        const StateVector propagated =
            helios::orbital::propagate_conic(inertial, k_earth_mu_m3_s2, days * 86'400.0).value();
        const auto spherical = to_spherical(earth.to_body_fixed(propagated, later).position_m);
        // Only the slow drift of the IAU pole (0.6°/century) moves the equator under the orbit.
        EXPECT_LT(std::abs(spherical.longitude_rad) / k_degree_rad, 1e-3) << days;
        EXPECT_LT(std::abs(spherical.latitude_rad) / k_degree_rad, 1e-3) << days;
    }
}

TEST(BodyRotation, RejectsNonFiniteElements) {
    EXPECT_FALSE(
        BodyRotation::make(RotationElements{.rotation_rate_rad_s = std::nan("")}, icrf_to_j2000_ecliptic())
            .has_value());
}

TEST(Spherical, LatitudeAndEastLongitude) {
    const auto point = to_spherical({0.0, 1.0, 1.0});
    EXPECT_NEAR(point.latitude_rad / k_degree_rad, 45.0, 1e-12);
    EXPECT_NEAR(point.longitude_rad / k_degree_rad, 90.0, 1e-12);
    EXPECT_NEAR(point.radius_m, std::sqrt(2.0), 1e-15);
}

} // namespace
