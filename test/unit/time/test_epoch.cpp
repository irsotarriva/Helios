#include "helios/time/epoch.hpp"

#include <cmath>
#include <cstdint>
#include <gtest/gtest.h>
#include <limits>

namespace {

using helios::core::ErrorCode;
using helios::time::Epoch;
using helios::time::Interval;
using helios::time::k_seconds_per_julian_year;
using helios::time::seconds_between;

TEST(Epoch, DefaultIsTheReferenceEpoch) {
    constexpr Epoch k_reference{};
    static_assert(k_reference.whole_seconds() == 0);
    static_assert(k_reference.fraction() == 0.0);
}

TEST(Epoch, FractionAboveOneCarriesForward) {
    const auto epoch = Epoch::from_parts(10, 2.5);
    ASSERT_TRUE(epoch.has_value());
    EXPECT_EQ(epoch->whole_seconds(), 12);
    EXPECT_EQ(epoch->fraction(), 0.5);
}

TEST(Epoch, NegativeFractionBorrows) {
    const auto epoch = Epoch::from_parts(10, -0.25);
    ASSERT_TRUE(epoch.has_value());
    EXPECT_EQ(epoch->whole_seconds(), 9);
    EXPECT_EQ(epoch->fraction(), 0.75);
}

TEST(Epoch, TinyNegativeFractionNeverRoundsToOne) {
    const auto epoch = Epoch::from_parts(0, -1e-20);
    ASSERT_TRUE(epoch.has_value());
    EXPECT_GE(epoch->fraction(), 0.0);
    EXPECT_LT(epoch->fraction(), 1.0);
}

TEST(Epoch, RejectsNonFiniteInput) {
    const auto nan_epoch = Epoch::from_parts(0, std::numeric_limits<double>::quiet_NaN());
    ASSERT_FALSE(nan_epoch.has_value());
    EXPECT_EQ(nan_epoch.error().code, ErrorCode::NotFinite);

    const auto infinite_epoch = Epoch::from_seconds(std::numeric_limits<double>::infinity());
    ASSERT_FALSE(infinite_epoch.has_value());
    EXPECT_EQ(infinite_epoch.error().code, ErrorCode::NotFinite);
}

TEST(Epoch, RejectsInt64Overflow) {
    const auto overflowing = Epoch::from_parts(std::numeric_limits<std::int64_t>::max(), 1.5);
    ASSERT_FALSE(overflowing.has_value());
    EXPECT_EQ(overflowing.error().code, ErrorCode::Overflow);

    const auto beyond_int64 = Epoch::from_seconds(1e19);
    ASSERT_FALSE(beyond_int64.has_value());
    EXPECT_EQ(beyond_int64.error().code, ErrorCode::Overflow);
}

TEST(Epoch, OrderingIsChronological) {
    const Epoch earlier = Epoch::from_parts(5, 0.5).value();
    const Epoch later = Epoch::from_parts(6, 0.1).value();
    EXPECT_LT(earlier, later);
    EXPECT_GT(later, earlier);
    EXPECT_EQ(earlier, Epoch::from_parts(4, 1.5).value());
}

TEST(Epoch, AdvancedByAndSecondsBetweenAreInverse) {
    const Epoch start = Epoch::from_parts(1'000'000'000'000, 0.125).value();
    for (const double offset : {0.0, 1e-9, 0.875, 1.0, 3600.5, -42.25, -1e-12, 3.0e9}) {
        const Epoch moved = start.advanced_by(offset).value();
        const double tolerance = 1e-15 + 1e-16 * std::abs(offset);
        EXPECT_NEAR(seconds_between(start, moved), offset, tolerance) << "offset " << offset;
        EXPECT_NEAR(seconds_between(moved, start), -offset, tolerance) << "offset " << offset;
    }
}

TEST(Epoch, SubNanosecondResolutionSurvivesABillionYears) {
    // A plain double at this epoch has a 4 s ULP; the whole point of Epoch (BRIEFING §3.2).
    const auto billion_years = static_cast<std::int64_t>(1e9 * k_seconds_per_julian_year);
    const Epoch start = Epoch::from_parts(billion_years, 0.0).value();

    Epoch clock = start;
    constexpr int k_steps = 1000;
    for (int step = 0; step < k_steps; ++step) {
        clock = clock.advanced_by(1e-9).value();
    }
    EXPECT_NEAR(seconds_between(start, clock), 1e-6, 1e-15);
}

TEST(Epoch, SecondsBetweenKeepsTheWholeSecondDifferenceExact) {
    const Epoch from = Epoch::from_parts(1'000'000'000'000'000, 0.25).value();
    const Epoch to = Epoch::from_parts(1'000'000'000'000'003, 0.75).value();
    EXPECT_EQ(seconds_between(from, to), 3.5);
}

TEST(Interval, IsHalfOpen) {
    const Epoch begin = Epoch::from_seconds(10.0).value();
    const Epoch end = Epoch::from_seconds(20.0).value();

    const auto interval = Interval::make(begin, end);
    ASSERT_TRUE(interval.has_value());
    EXPECT_TRUE(interval->contains(begin));
    EXPECT_TRUE(interval->contains(Epoch::from_seconds(19.999).value()));
    EXPECT_FALSE(interval->contains(end));
    EXPECT_EQ(interval->duration_seconds(), 10.0);
}

TEST(Interval, RejectsEndBeforeBegin) {
    const Epoch begin = Epoch::from_seconds(10.0).value();
    const Epoch end = Epoch::from_seconds(20.0).value();

    const auto reversed = Interval::make(end, begin);
    ASSERT_FALSE(reversed.has_value());
    EXPECT_EQ(reversed.error().code, ErrorCode::InvalidArgument);
}

} // namespace
