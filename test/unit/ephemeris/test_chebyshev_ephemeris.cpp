#include "helios/ephemeris/chebyshev_ephemeris.hpp"

#include <gtest/gtest.h>
#include <limits>
#include <vector>

namespace {

using helios::core::ErrorCode;
using helios::ephemeris::ChebyshevEphemeris;
using helios::time::Epoch;

constexpr double k_record_duration_s = 345'600.0; // 4 days, as DE440 uses for the Moon
constexpr double k_half_duration_s = 0.5 * k_record_duration_s;
constexpr double k_acceleration_m_s2 = 0.01;

// x(t) = ½·a·t² exactly, t measured from the ephemeris start; y = z = 0.
// In a record with midpoint m and half-width h: x = ½a(m + hτ)²
//   = ½a(m² + h²/2)·T0 + a·m·h·T1 + (a·h²/4)·T2.
[[nodiscard]] ChebyshevEphemeris uniformly_accelerating(std::size_t record_count) {
    std::vector<double> coefficients_m;
    for (std::size_t record = 0; record < record_count; ++record) {
        const double midpoint_s = (static_cast<double>(record) + 0.5) * k_record_duration_s;
        const std::vector<double> x_axis{
            0.5 * k_acceleration_m_s2
                * (midpoint_s * midpoint_s + 0.5 * k_half_duration_s * k_half_duration_s),
            k_acceleration_m_s2 * midpoint_s * k_half_duration_s,
            0.25 * k_acceleration_m_s2 * k_half_duration_s * k_half_duration_s};
        coefficients_m.insert(coefficients_m.end(), x_axis.begin(), x_axis.end());
        coefficients_m.insert(coefficients_m.end(), 6, 0.0); // y and z axes
    }
    return ChebyshevEphemeris::make(Epoch{}, k_record_duration_s, 3, std::move(coefficients_m)).value();
}

TEST(ChebyshevEphemeris, ReproducesAnalyticMotionAndDerivatives) {
    const ChebyshevEphemeris model = uniformly_accelerating(5);
    // The series sums terms up to ~x(end), so the floating-point floor is a few ulp of that.
    constexpr double k_position_scale_m =
        0.5 * k_acceleration_m_s2 * 25.0 * k_record_duration_s * k_record_duration_s;
    constexpr double k_position_tolerance_m =
        8.0 * std::numeric_limits<double>::epsilon() * k_position_scale_m;
    for (const double elapsed_s :
         {0.0, 1.0, 100'000.0, k_record_duration_s, 1'000'000.0, 5.0 * k_record_duration_s}) {
        const Epoch instant = Epoch::from_seconds(elapsed_s).value();
        const auto state = model.state_at(instant);
        ASSERT_TRUE(state.has_value()) << elapsed_s;
        const double expected_position_m = 0.5 * k_acceleration_m_s2 * elapsed_s * elapsed_s;
        EXPECT_NEAR(state->position_m.x, expected_position_m, k_position_tolerance_m) << elapsed_s;
        EXPECT_NEAR(state->velocity_m_s.x, k_acceleration_m_s2 * elapsed_s, 1e-9) << elapsed_s;
        EXPECT_NEAR(model.acceleration_at(instant).value().x, k_acceleration_m_s2, 1e-15) << elapsed_s;
        EXPECT_EQ(state->position_m.y, 0.0);
    }
}

TEST(ChebyshevEphemeris, CoverageIsClosedAndChecked) {
    const ChebyshevEphemeris model = uniformly_accelerating(2);
    ASSERT_TRUE(model.valid_range().has_value());
    EXPECT_EQ(model.record_count(), 2U);
    EXPECT_TRUE(model.state_at(Epoch::from_seconds(2.0 * k_record_duration_s).value()).has_value());
    EXPECT_EQ(model.state_at(Epoch::from_seconds(2.0 * k_record_duration_s + 1e-3).value()).error().code,
              ErrorCode::OutOfRange);
    EXPECT_EQ(model.state_at(Epoch::from_seconds(-1e-3).value()).error().code, ErrorCode::OutOfRange);
}

TEST(ChebyshevEphemeris, RejectsMalformedSeries) {
    EXPECT_EQ(ChebyshevEphemeris::make(Epoch{}, 0.0, 3, std::vector<double>(9, 0.0)).error().code,
              ErrorCode::OutOfRange);
    EXPECT_EQ(ChebyshevEphemeris::make(Epoch{}, 10.0, 0, std::vector<double>(9, 0.0)).error().code,
              ErrorCode::InvalidArgument);
    EXPECT_EQ(ChebyshevEphemeris::make(Epoch{}, 10.0, 3, std::vector<double>(10, 0.0)).error().code,
              ErrorCode::InvalidArgument);
    std::vector<double> with_nan(9, 0.0);
    with_nan[4] = std::nan("");
    EXPECT_EQ(ChebyshevEphemeris::make(Epoch{}, 10.0, 3, std::move(with_nan)).error().code,
              ErrorCode::NotFinite);
}

} // namespace
