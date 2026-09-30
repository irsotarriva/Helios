#include "helios/orbital/kepler.hpp"

#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <random>

namespace {

using helios::core::ErrorCode;
using helios::math::cross;
using helios::math::dot;
using helios::math::norm;
using helios::math::Vector3;
using helios::orbital::elements_from_state;
using helios::orbital::k_pi;
using helios::orbital::k_two_pi;
using helios::orbital::KeplerianElements;
using helios::orbital::solve_kepler_equation;
using helios::orbital::state_from_elements;
using helios::orbital::StateVector;
using helios::orbital::two_body_acceleration_m_s2;
using helios::orbital::wrap_angle_rad;

constexpr double k_earth_mu_m3_s2 = 3.986004418e14;

[[nodiscard]] double angle_difference_rad(double lhs_rad, double rhs_rad) {
    return std::abs(std::remainder(lhs_rad - rhs_rad, k_two_pi));
}

TEST(KeplerEquation, ResidualIsAtMachinePrecisionAcrossEccentricities) {
    for (const double eccentricity : {0.0, 1e-8, 0.1, 0.5, 0.9, 0.99, 0.999}) {
        for (int step = -400; step <= 400; ++step) {
            const double mean_anomaly_rad = 0.01 * static_cast<double>(step) * k_pi;
            const auto eccentric_anomaly_rad = solve_kepler_equation(mean_anomaly_rad, eccentricity);
            ASSERT_TRUE(eccentric_anomaly_rad.has_value())
                << "e=" << eccentricity << " M=" << mean_anomaly_rad;
            const double residual_rad =
                *eccentric_anomaly_rad - eccentricity * std::sin(*eccentric_anomaly_rad) - mean_anomaly_rad;
            EXPECT_LT(std::abs(residual_rad), 1e-13) << "e=" << eccentricity << " M=" << mean_anomaly_rad;
        }
    }
}

TEST(KeplerEquation, RejectsInvalidEccentricityAndNonFinite) {
    EXPECT_EQ(solve_kepler_equation(1.0, 1.0).error().code, ErrorCode::OutOfRange);
    EXPECT_EQ(solve_kepler_equation(1.0, -0.1).error().code, ErrorCode::OutOfRange);
    EXPECT_EQ(solve_kepler_equation(std::numeric_limits<double>::quiet_NaN(), 0.1).error().code,
              ErrorCode::NotFinite);
}

TEST(WrapAngle, MapsIntoZeroToTwoPi) {
    EXPECT_NEAR(wrap_angle_rad(-0.5), k_two_pi - 0.5, 1e-15);
    EXPECT_NEAR(wrap_angle_rad(7.0), 7.0 - k_two_pi, 1e-15);
    EXPECT_EQ(wrap_angle_rad(0.0), 0.0);
}

TEST(StateFromElements, CircularOrbitHasCircularSpeed) {
    const KeplerianElements elements{.semi_major_axis_m = 7.0e6, .mean_anomaly_rad = 1.234};
    const auto state = state_from_elements(elements, k_earth_mu_m3_s2);
    ASSERT_TRUE(state.has_value());
    EXPECT_NEAR(norm(state->position_m), 7.0e6, 1e-6);
    EXPECT_NEAR(norm(state->velocity_m_s), std::sqrt(k_earth_mu_m3_s2 / 7.0e6), 1e-9);
}

TEST(StateFromElements, SatisfiesVisVivaAngularMomentumAndOrientation) {
    const KeplerianElements base{.semi_major_axis_m = 2.4e7,
                                 .eccentricity = 0.7,
                                 .inclination_rad = 0.9,
                                 .longitude_of_ascending_node_rad = 2.1,
                                 .argument_of_periapsis_rad = 4.0};
    const Vector3 expected_normal{std::sin(0.9) * std::sin(2.1), -std::sin(0.9) * std::cos(2.1),
                                  std::cos(0.9)};
    const double expected_angular_momentum_m2_s = std::sqrt(k_earth_mu_m3_s2 * 2.4e7 * (1.0 - 0.49));

    for (int step = 0; step < 64; ++step) {
        KeplerianElements elements = base;
        elements.mean_anomaly_rad = k_two_pi * static_cast<double>(step) / 64.0;
        const StateVector state = state_from_elements(elements, k_earth_mu_m3_s2).value();

        const double radius_m = norm(state.position_m);
        const double vis_viva_m2_s2 = k_earth_mu_m3_s2 * (2.0 / radius_m - 1.0 / 2.4e7);
        EXPECT_NEAR(dot(state.velocity_m_s, state.velocity_m_s) / vis_viva_m2_s2, 1.0, 1e-13);

        const Vector3 angular_momentum_m2_s = cross(state.position_m, state.velocity_m_s);
        EXPECT_NEAR(norm(angular_momentum_m2_s) / expected_angular_momentum_m2_s, 1.0, 1e-13);
        EXPECT_NEAR(dot(angular_momentum_m2_s / norm(angular_momentum_m2_s), expected_normal), 1.0, 1e-13);
    }

    const StateVector periapsis = state_from_elements(base, k_earth_mu_m3_s2).value();
    EXPECT_NEAR(norm(periapsis.position_m), 2.4e7 * 0.3, 1e-6);
}

TEST(StateFromElements, RejectsUnphysicalInput) {
    EXPECT_EQ(state_from_elements({.semi_major_axis_m = -1.0}, k_earth_mu_m3_s2).error().code,
              ErrorCode::OutOfRange);
    EXPECT_EQ(
        state_from_elements({.semi_major_axis_m = 1.0, .eccentricity = 1.2}, k_earth_mu_m3_s2).error().code,
        ErrorCode::OutOfRange);
    EXPECT_EQ(state_from_elements({.semi_major_axis_m = 1.0}, 0.0).error().code, ErrorCode::OutOfRange);
}

TEST(ElementsFromState, RoundTripsGenericOrbits) {
    std::mt19937_64 generator(20260930);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    for (int trial = 0; trial < 500; ++trial) {
        const KeplerianElements elements{.semi_major_axis_m = 7e6 + 4e8 * unit(generator),
                                         .eccentricity = 0.001 + 0.95 * unit(generator),
                                         .inclination_rad = 0.01 + 3.1 * unit(generator),
                                         .longitude_of_ascending_node_rad = k_two_pi * unit(generator),
                                         .argument_of_periapsis_rad = k_two_pi * unit(generator),
                                         .mean_anomaly_rad = k_two_pi * unit(generator)};
        const StateVector state = state_from_elements(elements, k_earth_mu_m3_s2).value();
        const KeplerianElements recovered = elements_from_state(state, k_earth_mu_m3_s2).value();

        EXPECT_NEAR(recovered.semi_major_axis_m / elements.semi_major_axis_m, 1.0, 1e-11);
        EXPECT_NEAR(recovered.eccentricity, elements.eccentricity, 1e-11);
        EXPECT_NEAR(recovered.inclination_rad, elements.inclination_rad, 1e-11);
        EXPECT_LT(angle_difference_rad(recovered.longitude_of_ascending_node_rad,
                                       elements.longitude_of_ascending_node_rad),
                  1e-10);
        EXPECT_LT(
            angle_difference_rad(recovered.argument_of_periapsis_rad, elements.argument_of_periapsis_rad),
            1e-9);
        EXPECT_LT(angle_difference_rad(recovered.mean_anomaly_rad, elements.mean_anomaly_rad), 1e-9);
    }
}

TEST(ElementsFromState, DegenerateOrbitsFollowConventionsAndRoundTripTheState) {
    // Circular equatorial, prograde and retrograde: Ω = ω = 0, anomaly measured from +x.
    for (const double inclination_rad : {0.0, k_pi}) {
        const KeplerianElements elements{
            .semi_major_axis_m = 7e6, .inclination_rad = inclination_rad, .mean_anomaly_rad = 1.0};
        const StateVector state = state_from_elements(elements, k_earth_mu_m3_s2).value();
        const KeplerianElements recovered = elements_from_state(state, k_earth_mu_m3_s2).value();
        EXPECT_EQ(recovered.longitude_of_ascending_node_rad, 0.0);
        EXPECT_EQ(recovered.argument_of_periapsis_rad, 0.0);
        const StateVector again = state_from_elements(recovered, k_earth_mu_m3_s2).value();
        EXPECT_LT(norm(again.position_m - state.position_m), 1e-6);
        EXPECT_LT(norm(again.velocity_m_s - state.velocity_m_s), 1e-9);
    }
}

TEST(ElementsFromState, RejectsUnboundAndRadialStates) {
    const StateVector escaping{.position_m = {7e6, 0.0, 0.0}, .velocity_m_s = {0.0, 20'000.0, 0.0}};
    EXPECT_EQ(elements_from_state(escaping, k_earth_mu_m3_s2).error().code, ErrorCode::OutOfRange);

    const StateVector radial{.position_m = {7e6, 0.0, 0.0}, .velocity_m_s = {100.0, 0.0, 0.0}};
    EXPECT_EQ(elements_from_state(radial, k_earth_mu_m3_s2).error().code, ErrorCode::InvalidArgument);
}

TEST(TwoBodyAcceleration, PointsInwardWithInverseSquareMagnitude) {
    const Vector3 acceleration_m_s2 = two_body_acceleration_m_s2({0.0, 7e6, 0.0}, k_earth_mu_m3_s2);
    EXPECT_DOUBLE_EQ(acceleration_m_s2.y, -k_earth_mu_m3_s2 / 49e12);
    EXPECT_EQ(acceleration_m_s2.x, 0.0);
}

} // namespace
