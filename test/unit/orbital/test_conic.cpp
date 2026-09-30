#include "helios/orbital/conic.hpp"

#include <cmath>
#include <gtest/gtest.h>

namespace {

using helios::math::dot;
using helios::math::norm;
using helios::math::Vector3;
using helios::orbital::conic_geometry;
using helios::orbital::ConicGeometry;
using helios::orbital::k_pi;
using helios::orbital::maneuver_basis;
using helios::orbital::maneuver_to_inertial;
using helios::orbital::orbital_period_s;
using helios::orbital::propagate_conic;
using helios::orbital::RadialDirection;
using helios::orbital::sample_conic;
using helios::orbital::StateVector;
using helios::orbital::time_since_periapsis_s;
using helios::orbital::time_to_apoapsis_s;
using helios::orbital::time_to_periapsis_s;
using helios::orbital::time_to_radius_s;
using helios::orbital::true_anomaly_limit_rad;

constexpr double k_earth_mu_m3_s2 = 3.986004418e14;

// Rotate by inclination about x, then by node about z, so the tests never sit in a special plane.
[[nodiscard]] Vector3 tilt(const Vector3& vector) {
    constexpr double k_inclination_rad = 0.7;
    constexpr double k_node_rad = 2.1;
    const Vector3 about_x{vector.x,
                          std::cos(k_inclination_rad) * vector.y - std::sin(k_inclination_rad) * vector.z,
                          std::sin(k_inclination_rad) * vector.y + std::cos(k_inclination_rad) * vector.z};
    return {std::cos(k_node_rad) * about_x.x - std::sin(k_node_rad) * about_x.y,
            std::sin(k_node_rad) * about_x.x + std::cos(k_node_rad) * about_x.y, about_x.z};
}

// State at true anomaly ν on the conic with periapsis radius r_p and eccentricity e.
[[nodiscard]] StateVector state_on_conic(double periapsis_radius_m, double eccentricity,
                                         double true_anomaly_rad) {
    const double semi_latus_rectum_m = periapsis_radius_m * (1.0 + eccentricity);
    const double radius_m = semi_latus_rectum_m / (1.0 + eccentricity * std::cos(true_anomaly_rad));
    const double speed_scale_m_s = std::sqrt(k_earth_mu_m3_s2 / semi_latus_rectum_m);
    return StateVector{
        .position_m =
            tilt({radius_m * std::cos(true_anomaly_rad), radius_m * std::sin(true_anomaly_rad), 0.0}),
        .velocity_m_s = tilt({-speed_scale_m_s * std::sin(true_anomaly_rad),
                              speed_scale_m_s * (eccentricity + std::cos(true_anomaly_rad)), 0.0})};
}

[[nodiscard]] ConicGeometry geometry_of(const StateVector& state) {
    auto conic = conic_geometry(state, k_earth_mu_m3_s2);
    EXPECT_TRUE(conic.has_value());
    return *conic;
}

[[nodiscard]] StateVector advance(const StateVector& state, double elapsed_s) {
    auto advanced = propagate_conic(state, k_earth_mu_m3_s2, elapsed_s);
    EXPECT_TRUE(advanced.has_value());
    return *advanced;
}

struct ConicCase {
    double eccentricity;
    double true_anomaly_rad;
};

class ConicTiming : public ::testing::TestWithParam<ConicCase> {};

TEST_P(ConicTiming, GeometryRecoversTheConstruction) {
    const auto [eccentricity, true_anomaly_rad] = GetParam();
    const ConicGeometry conic = geometry_of(state_on_conic(7.0e6, eccentricity, true_anomaly_rad));
    EXPECT_NEAR(conic.eccentricity, eccentricity, 1e-12);
    EXPECT_NEAR(conic.periapsis_radius_m, 7.0e6, 1e-5);
    EXPECT_NEAR(conic.true_anomaly_rad, true_anomaly_rad, 1e-11);
    EXPECT_NEAR(dot(conic.periapsis_unit, conic.normal_unit), 0.0, 1e-15);
}

TEST_P(ConicTiming, PeriapsisTimeAgreesWithUniversalVariablePropagation) {
    const auto [eccentricity, true_anomaly_rad] = GetParam();
    const StateVector state = state_on_conic(7.0e6, eccentricity, true_anomaly_rad);
    const ConicGeometry conic = geometry_of(state);
    const auto until_s = time_to_periapsis_s(conic);
    if (eccentricity >= 1.0 && true_anomaly_rad > 0.0) {
        EXPECT_FALSE(until_s.has_value()) << "an escaping conic past periapsis has no next periapsis";
        return;
    }
    ASSERT_TRUE(until_s.has_value());
    EXPECT_GT(*until_s, 0.0);
    const StateVector at_periapsis = advance(state, *until_s);
    EXPECT_NEAR(norm(at_periapsis.position_m) / 7.0e6, 1.0, 1e-9);
    EXPECT_NEAR(dot(at_periapsis.position_m, at_periapsis.velocity_m_s)
                    / (norm(at_periapsis.position_m) * norm(at_periapsis.velocity_m_s)),
                0.0, 1e-7);
}

TEST_P(ConicTiming, RadiusCrossingTimesLandOnTheRadiusWithTheRightSense) {
    const auto [eccentricity, true_anomaly_rad] = GetParam();
    const StateVector state = state_on_conic(7.0e6, eccentricity, true_anomaly_rad);
    const ConicGeometry conic = geometry_of(state);
    const double target_m = eccentricity < 1.0 ? 0.5 * (conic.periapsis_radius_m + conic.apoapsis_radius_m)
                                               : 5.0 * conic.periapsis_radius_m;
    for (const RadialDirection direction : {RadialDirection::Outbound, RadialDirection::Inbound}) {
        const auto until_s = time_to_radius_s(conic, target_m, direction);
        const bool outbound = direction == RadialDirection::Outbound;
        if (eccentricity >= 1.0 && !outbound) {
            // Inbound crossing of 5 r_p happens before periapsis; it is in the future only if
            // the vessel is still outside that radius.
            if (norm(state.position_m) < target_m) {
                EXPECT_FALSE(until_s.has_value());
                continue;
            }
        }
        ASSERT_TRUE(until_s.has_value()) << "outbound=" << outbound;
        EXPECT_GT(*until_s, 0.0);
        if (eccentricity < 1.0) {
            EXPECT_LE(*until_s, orbital_period_s(conic));
        }
        const StateVector crossing = advance(state, *until_s);
        EXPECT_NEAR(norm(crossing.position_m) / target_m, 1.0, 1e-9);
        const double radial_speed_m_s =
            dot(crossing.position_m, crossing.velocity_m_s) / norm(crossing.position_m);
        EXPECT_EQ(radial_speed_m_s > 0.0, outbound);
    }
}

INSTANTIATE_TEST_SUITE_P(Conics, ConicTiming,
                         ::testing::Values(ConicCase{0.01, 1.0}, ConicCase{0.3, -2.0}, ConicCase{0.3, 2.5},
                                           ConicCase{0.97, 3.0}, ConicCase{1.8, -1.5}, ConicCase{1.8, 0.4},
                                           ConicCase{4.0, -1.7}));

TEST(ConicTiming, ApoapsisIsHalfAPeriodAfterPeriapsis) {
    const StateVector state = state_on_conic(7.0e6, 0.4, 0.9);
    const ConicGeometry conic = geometry_of(state);
    const auto periapsis_s = time_to_periapsis_s(conic);
    const auto apoapsis_s = time_to_apoapsis_s(conic);
    ASSERT_TRUE(periapsis_s.has_value() && apoapsis_s.has_value());
    const double period_s = orbital_period_s(conic);
    EXPECT_NEAR(std::remainder(*periapsis_s - *apoapsis_s, period_s),
                0.5 * period_s * (*periapsis_s > *apoapsis_s ? 1 : -1), 1e-6);
    const StateVector at_apoapsis = advance(state, *apoapsis_s);
    EXPECT_NEAR(norm(at_apoapsis.position_m) / conic.apoapsis_radius_m, 1.0, 1e-10);
}

TEST(ConicTiming, NearParabolicUsesBarkerAndStaysContinuous) {
    // The elliptic, parabolic and hyperbolic branches must agree across e = 1.
    const double below = time_since_periapsis_s(geometry_of(state_on_conic(7.0e6, 1.0 - 2e-8, 0.0)), 2.0);
    const double at = time_since_periapsis_s(geometry_of(state_on_conic(7.0e6, 1.0, 0.0)), 2.0);
    const double above = time_since_periapsis_s(geometry_of(state_on_conic(7.0e6, 1.0 + 2e-8, 0.0)), 2.0);
    EXPECT_NEAR(below / at, 1.0, 1e-6);
    EXPECT_NEAR(above / at, 1.0, 1e-6);
}

TEST(ConicTiming, UnreachableRadiiAndCircularOrbitsReportNone) {
    const ConicGeometry ellipse = geometry_of(state_on_conic(7.0e6, 0.2, 0.3));
    EXPECT_FALSE(time_to_radius_s(ellipse, 6.0e6, RadialDirection::Inbound).has_value());
    EXPECT_FALSE(time_to_radius_s(ellipse, 1.0e9, RadialDirection::Outbound).has_value());
    const ConicGeometry circle = geometry_of(state_on_conic(7.0e6, 0.0, 0.3));
    EXPECT_FALSE(time_to_periapsis_s(circle).has_value());
    EXPECT_FALSE(time_to_radius_s(circle, 7.5e6, RadialDirection::Outbound).has_value());
}

TEST(ConicTiming, RejectsDegenerateStates) {
    EXPECT_FALSE(
        conic_geometry(StateVector{.position_m = {7e6, 0, 0}, .velocity_m_s = {1e3, 0, 0}}, k_earth_mu_m3_s2)
            .has_value());
    EXPECT_FALSE(conic_geometry(StateVector{}, k_earth_mu_m3_s2).has_value());
}

TEST(SampleConic, ClosedEllipsePointsLieOnTheConic) {
    const ConicGeometry conic = geometry_of(state_on_conic(7.0e6, 0.6, 1.1));
    const auto arc = sample_conic(conic, 1e12, 128);
    ASSERT_TRUE(arc.has_value());
    EXPECT_TRUE(arc->closed);
    ASSERT_EQ(arc->points_m.size(), 128U);
    for (const Vector3& point_m : arc->points_m) {
        const double true_anomaly_rad =
            std::atan2(dot(point_m, conic.in_plane_unit), dot(point_m, conic.periapsis_unit));
        const double expected_radius_m =
            conic.semi_latus_rectum_m / (1.0 + conic.eccentricity * std::cos(true_anomaly_rad));
        EXPECT_NEAR(norm(point_m) / expected_radius_m, 1.0, 1e-12);
        EXPECT_NEAR(dot(point_m, conic.normal_unit) / norm(point_m), 0.0, 1e-14);
    }
}

TEST(SampleConic, OpenArcsEndOnTheClipSphere) {
    for (const double eccentricity : {0.99, 1.0, 2.5}) {
        const ConicGeometry conic = geometry_of(state_on_conic(7.0e6, eccentricity, 0.2));
        constexpr double k_clip_m = 9.0e8;
        const auto arc = sample_conic(conic, k_clip_m, 65);
        ASSERT_TRUE(arc.has_value());
        EXPECT_FALSE(arc->closed);
        EXPECT_NEAR(norm(arc->points_m.front()) / k_clip_m, 1.0, 1e-9) << "e=" << eccentricity;
        EXPECT_NEAR(norm(arc->points_m.back()) / k_clip_m, 1.0, 1e-9) << "e=" << eccentricity;
        EXPECT_NEAR(norm(arc->points_m[32]) / conic.periapsis_radius_m, 1.0, 1e-12) << "e=" << eccentricity;
        for (const Vector3& point_m : arc->points_m) {
            EXPECT_LE(norm(point_m), k_clip_m * (1.0 + 1e-9));
        }
    }
    EXPECT_LT(true_anomaly_limit_rad(2.5), k_pi);
}

TEST(SampleConic, RejectsTooFewPointsOrAClipInsidePeriapsis) {
    const ConicGeometry conic = geometry_of(state_on_conic(7.0e6, 0.1, 0.0));
    EXPECT_FALSE(sample_conic(conic, 1e9, 3).has_value());
    EXPECT_FALSE(sample_conic(conic, 1e6, 16).has_value());
}

TEST(ManeuverBasis, IsOrthonormalAndRadialOutPointsAway) {
    const StateVector state = state_on_conic(7.0e6, 0.0, 0.8);
    const auto basis = maneuver_basis(state);
    ASSERT_TRUE(basis.has_value());
    EXPECT_NEAR(norm(basis->prograde_unit), 1.0, 1e-15);
    EXPECT_NEAR(norm(basis->radial_out_unit), 1.0, 1e-15);
    EXPECT_NEAR(dot(basis->prograde_unit, basis->normal_unit), 0.0, 1e-15);
    EXPECT_NEAR(dot(basis->radial_out_unit, state.position_m) / norm(state.position_m), 1.0, 1e-12);
    const Vector3 delta_v_m_s = maneuver_to_inertial(*basis, 3.0, -4.0, 12.0);
    EXPECT_NEAR(norm(delta_v_m_s), 13.0, 1e-12);
    EXPECT_NEAR(dot(delta_v_m_s, basis->normal_unit), -4.0, 1e-12);
}

} // namespace
