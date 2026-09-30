#include "helios/sim/time_warp.hpp"

#include <gtest/gtest.h>

namespace {

using helios::sim::k_real_time_level;
using helios::sim::k_warp_levels;
using helios::sim::TimeWarp;
using helios::time::Epoch;
using helios::time::seconds_between;

[[nodiscard]] Epoch at(double seconds) {
    return Epoch::from_seconds(seconds).value();
}

TEST(TimeWarp, StartsAtRealTimeAndClampsLevels) {
    TimeWarp warp;
    EXPECT_EQ(warp.requested_factor(), 1.0);
    warp.request_level(99);
    EXPECT_EQ(warp.requested_factor(), 1e6);
    warp.increase();
    EXPECT_EQ(warp.requested_factor(), 1e6);
    warp.request_level(0);
    warp.decrease();
    EXPECT_EQ(warp.requested_factor(), 0.01);
}

TEST(TimeWarp, EventsLimitTheFactorToKeepTwoWallSecondsOfLead) {
    TimeWarp warp;
    warp.request_level(k_warp_levels.size() - 1); // 1e6×
    EXPECT_EQ(warp.limited_factor(std::nullopt), 1e6);
    EXPECT_EQ(warp.limited_factor(3e7), 1e6);
    EXPECT_EQ(warp.limited_factor(1e5), 1e4); // 1e5 s / 2 s = 5e4× allowed → the 1e4 level
    EXPECT_EQ(warp.limited_factor(0.5), 1.0); // never below real time
    warp.request_level(1);                    // 0.1× slow motion is never limited
    EXPECT_EQ(warp.limited_factor(0.5), 0.1);
}

TEST(TimeWarp, AdvanceLandsExactlyOnTheNextEventAndNeverPassesIt) {
    TimeWarp warp;
    warp.request_level(k_warp_levels.size() - 1);
    const Epoch now = at(1'000.0);
    const Epoch event = at(1'000.0 + 30.0);
    const auto step = warp.advance(now, 1.0 / 60.0, event).value();
    EXPECT_EQ(step.factor, 10.0); // 30 s / 2 s = 15× allowed → 10×
    EXPECT_NEAR(seconds_between(now, step.epoch), 10.0 / 60.0, 1e-12);
    const auto landing = warp.advance(at(1'029.99), 1.0, event).value();
    EXPECT_EQ(landing.epoch, event);
    // A past event is ignored; the event we are sitting on allows only real time.
    EXPECT_EQ(warp.advance(event, 1.0, at(10.0)).value().factor, 1e6);
    EXPECT_EQ(warp.advance(event, 1.0, event).value().factor, 1.0);
}

TEST(TimeWarp, PausedDoesNotMoveAndBadFrameTimesFail) {
    TimeWarp warp;
    warp.set_paused(true);
    const auto step = warp.advance(at(5.0), 0.1, std::nullopt).value();
    EXPECT_EQ(step.epoch, at(5.0));
    EXPECT_EQ(step.factor, 0.0);
    EXPECT_FALSE(warp.advance(at(5.0), -1.0, std::nullopt).has_value());
    EXPECT_EQ(k_warp_levels.at(k_real_time_level), 1.0);
}

} // namespace
