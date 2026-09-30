#include "helios/ephemeris/keplerian_ephemeris.hpp"

#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>

namespace {

using helios::core::ErrorCode;
using helios::ephemeris::KeplerianEphemeris;
using helios::ephemeris::SecularElements;
using helios::math::norm;
using helios::orbital::k_two_pi;
using helios::time::Epoch;

// Roughly Mars, with exaggerated secular rates so the test exercises them.
[[nodiscard]] SecularElements mars_like() {
    return SecularElements{.reference_epoch = Epoch{},
                           .semi_major_axis_m = 2.2794e11,
                           .semi_major_axis_rate_m_s = 1e-3,
                           .eccentricity = 0.0934,
                           .eccentricity_rate_per_s = 1e-15,
                           .inclination_rad = 0.0323,
                           .inclination_rate_rad_s = -1e-14,
                           .longitude_of_ascending_node_rad = 0.865,
                           .longitude_of_ascending_node_rate_rad_s = -5e-12,
                           .longitude_of_periapsis_rad = 5.866,
                           .longitude_of_periapsis_rate_rad_s = 7e-12,
                           .mean_longitude_rad = 6.204,
                           .mean_longitude_rate_rad_s = 1.0586e-7};
}

TEST(KeplerianEphemeris, RejectsInvalidElements) {
    SecularElements elements = mars_like();
    elements.eccentricity = 1.0;
    EXPECT_EQ(KeplerianEphemeris::make(elements).error().code, ErrorCode::OutOfRange);
    elements = mars_like();
    elements.mean_longitude_rate_rad_s = 0.0;
    EXPECT_EQ(KeplerianEphemeris::make(elements).error().code, ErrorCode::OutOfRange);
    elements = mars_like();
    elements.inclination_rad = std::nan("");
    EXPECT_EQ(KeplerianEphemeris::make(elements).error().code, ErrorCode::NotFinite);
}

[[nodiscard]] SecularElements without_secular_drift(SecularElements elements) {
    elements.semi_major_axis_rate_m_s = 0.0;
    elements.eccentricity_rate_per_s = 0.0;
    elements.inclination_rate_rad_s = 0.0;
    elements.longitude_of_ascending_node_rate_rad_s = 0.0;
    elements.longitude_of_periapsis_rate_rad_s = 0.0;
    return elements;
}

[[nodiscard]] double max_acceleration_mismatch(const KeplerianEphemeris& model) {
    constexpr double k_step_s = 600.0;
    double worst_relative_error = 0.0;
    for (const double days : {0.0, 123.4, 5000.0, -20000.0}) {
        const Epoch instant = Epoch::from_seconds(days * 86'400.0).value();
        const auto before = model.state_at(instant.advanced_by(-k_step_s).value()).value();
        const auto after = model.state_at(instant.advanced_by(k_step_s).value()).value();
        const auto finite_difference_m_s2 = (after.velocity_m_s - before.velocity_m_s) / (2.0 * k_step_s);
        const auto analytic_m_s2 = model.acceleration_at(instant).value();
        worst_relative_error = std::max(worst_relative_error,
                                        norm(finite_difference_m_s2 - analytic_m_s2) / norm(analytic_m_s2));
    }
    return worst_relative_error;
}

TEST(KeplerianEphemeris, AccelerationIsTheDerivativeOfVelocity) {
    // Exact Kepler motion: only finite-difference truncation (≈ (n·h)²/6 ≈ 6e-10) remains.
    EXPECT_LT(max_acceleration_mismatch(KeplerianEphemeris::make(without_secular_drift(mars_like())).value()),
              1e-8);
    // With (exaggerated) secular rates the documented bound is |ϖ̇|/n ≈ 7e-5.
    EXPECT_LT(max_acceleration_mismatch(KeplerianEphemeris::make(mars_like()).value()), 1e-4);
}

TEST(KeplerianEphemeris, VelocityIsTheDerivativeOfPosition) {
    const KeplerianEphemeris model = KeplerianEphemeris::make(mars_like()).value();
    constexpr double k_step_s = 60.0;
    const Epoch instant = Epoch::from_seconds(1.0e8).value();
    const auto before = model.state_at(instant.advanced_by(-k_step_s).value()).value();
    const auto after = model.state_at(instant.advanced_by(k_step_s).value()).value();
    const auto finite_difference_m_s = (after.position_m - before.position_m) / (2.0 * k_step_s);
    const auto analytic_m_s = model.state_at(instant).value().velocity_m_s;
    // Element drift adds a relative error of order |ϖ̇|/n ≈ 1e-4 with these exaggerated rates.
    EXPECT_LT(norm(finite_difference_m_s - analytic_m_s) / norm(analytic_m_s), 2e-4);
}

TEST(KeplerianEphemeris, OrbitClosesAfterOnePeriodWithoutSecularRates) {
    const SecularElements elements = without_secular_drift(mars_like());
    const KeplerianEphemeris model = KeplerianEphemeris::make(elements).value();

    const double period_s = k_two_pi / elements.mean_longitude_rate_rad_s;
    const Epoch start = Epoch::from_seconds(3.3e7).value();
    const auto first = model.state_at(start).value();
    const auto one_period_later = model.state_at(start.advanced_by(100.0 * period_s).value()).value();
    // 100 orbits ≈ 188 years: only the rounding of n·Δt remains (well under a metre).
    EXPECT_LT(norm(one_period_later.position_m - first.position_m), 1.0);
}

TEST(KeplerianEphemeris, IsUnboundedInTime) {
    const KeplerianEphemeris model = KeplerianEphemeris::make(mars_like()).value();
    EXPECT_FALSE(model.valid_range().has_value());
}

} // namespace
