#include "helios/time/epoch.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstdint>
#include <limits>

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using helios::core::ErrorCode;
using helios::time::Epoch;
using helios::time::Interval;
using helios::time::k_seconds_per_julian_year;
using helios::time::seconds_between;

TEST_CASE("Default epoch is the reference epoch", "[time][epoch]") {
    constexpr Epoch k_reference{};
    STATIC_REQUIRE(k_reference.whole_seconds() == 0);
    STATIC_REQUIRE(k_reference.fraction() == 0.0);
}

TEST_CASE("from_parts normalises the fraction to the unit interval", "[time][epoch]") {
    SECTION("fraction above one carries forward") {
        const auto epoch = Epoch::from_parts(10, 2.5);
        REQUIRE(epoch.has_value());
        CHECK(epoch->whole_seconds() == 12);
        CHECK(epoch->fraction() == 0.5);
    }
    SECTION("negative fraction borrows") {
        const auto epoch = Epoch::from_parts(10, -0.25);
        REQUIRE(epoch.has_value());
        CHECK(epoch->whole_seconds() == 9);
        CHECK(epoch->fraction() == 0.75);
    }
    SECTION("tiny negative fraction never produces fraction == 1") {
        const auto epoch = Epoch::from_parts(0, -1e-20);
        REQUIRE(epoch.has_value());
        CHECK(epoch->fraction() >= 0.0);
        CHECK(epoch->fraction() < 1.0);
    }
}

TEST_CASE("Non-finite and overflowing inputs are rejected", "[time][epoch]") {
    const auto nan_epoch = Epoch::from_parts(0, std::numeric_limits<double>::quiet_NaN());
    REQUIRE_FALSE(nan_epoch.has_value());
    CHECK(nan_epoch.error().code == ErrorCode::NotFinite);

    const auto infinite_epoch = Epoch::from_seconds(std::numeric_limits<double>::infinity());
    REQUIRE_FALSE(infinite_epoch.has_value());
    CHECK(infinite_epoch.error().code == ErrorCode::NotFinite);

    const auto overflowing = Epoch::from_parts(std::numeric_limits<std::int64_t>::max(), 1.5);
    REQUIRE_FALSE(overflowing.has_value());
    CHECK(overflowing.error().code == ErrorCode::Overflow);

    const auto beyond_int64 = Epoch::from_seconds(1e19);
    REQUIRE_FALSE(beyond_int64.has_value());
    CHECK(beyond_int64.error().code == ErrorCode::Overflow);
}

TEST_CASE("Ordering is chronological", "[time][epoch]") {
    const Epoch earlier = Epoch::from_parts(5, 0.5).value();
    const Epoch later = Epoch::from_parts(6, 0.1).value();
    CHECK(earlier < later);
    CHECK(later > earlier);
    CHECK(earlier == Epoch::from_parts(4, 1.5).value());
}

TEST_CASE("advanced_by and seconds_between are inverse", "[time][epoch]") {
    const Epoch start = Epoch::from_parts(1'000'000'000'000, 0.125).value();
    for (const double offset : {0.0, 1e-9, 0.875, 1.0, 3600.5, -42.25, -1e-12, 3.0e9}) {
        const Epoch moved = start.advanced_by(offset).value();
        CHECK_THAT(seconds_between(start, moved), WithinAbs(offset, 1e-15 + 1e-16 * std::abs(offset)));
        CHECK_THAT(seconds_between(moved, start), WithinAbs(-offset, 1e-15 + 1e-16 * std::abs(offset)));
    }
}

TEST_CASE("Sub-nanosecond resolution survives a billion years", "[time][epoch]") {
    // A plain double at this epoch has a 4 s ULP; the whole point of Epoch (BRIEFING §3.2).
    const auto billion_years = static_cast<std::int64_t>(1e9 * k_seconds_per_julian_year);
    const Epoch start = Epoch::from_parts(billion_years, 0.0).value();

    Epoch clock = start;
    constexpr int k_steps = 1000;
    for (int step = 0; step < k_steps; ++step) {
        clock = clock.advanced_by(1e-9).value();
    }
    CHECK_THAT(seconds_between(start, clock), WithinRel(1e-6, 1e-9));
}

TEST_CASE("seconds_between keeps the whole-second difference exact", "[time][epoch]") {
    const Epoch from = Epoch::from_parts(1'000'000'000'000'000, 0.25).value();
    const Epoch to = Epoch::from_parts(1'000'000'000'000'003, 0.75).value();
    CHECK(seconds_between(from, to) == 3.5);
}

TEST_CASE("Interval is half-open and validated", "[time][interval]") {
    const Epoch begin = Epoch::from_seconds(10.0).value();
    const Epoch end = Epoch::from_seconds(20.0).value();

    const auto interval = Interval::make(begin, end);
    REQUIRE(interval.has_value());
    CHECK(interval->contains(begin));
    CHECK(interval->contains(Epoch::from_seconds(19.999).value()));
    CHECK_FALSE(interval->contains(end));
    CHECK(interval->duration_seconds() == 10.0);

    const auto reversed = Interval::make(end, begin);
    REQUIRE_FALSE(reversed.has_value());
    CHECK(reversed.error().code == ErrorCode::InvalidArgument);
}
