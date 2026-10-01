#include "helios/vessel/control_bus.hpp"

#include <gtest/gtest.h>
#include <limits>

namespace {

using helios::core::ErrorCode;
using helios::vessel::ControlBus;
using helios::vessel::ControlSource;
using helios::vessel::SignalId;

TEST(ControlBus, ThePilotOverridesTheAutopilotUntilReleasing) {
    ControlBus bus;
    const SignalId throttle = bus.add_command("throttle", "", {.default_value = 0.25}).value();
    EXPECT_EQ(bus.value(throttle), 0.25);
    ASSERT_TRUE(bus.publish(throttle, ControlSource::Autopilot, 0.6).has_value());
    EXPECT_EQ(bus.value(throttle), 0.6);
    ASSERT_TRUE(bus.publish(throttle, ControlSource::Pilot, 0.1).has_value());
    EXPECT_EQ(bus.value(throttle), 0.1);
    // The autopilot keeps commanding underneath, and takes over again when the pilot lets go.
    ASSERT_TRUE(bus.publish(throttle, ControlSource::Autopilot, 0.9).has_value());
    EXPECT_EQ(bus.value(throttle), 0.1);
    ASSERT_TRUE(bus.release(throttle, ControlSource::Pilot).has_value());
    EXPECT_EQ(bus.value(throttle), 0.9);
    ASSERT_TRUE(bus.release(throttle, ControlSource::Autopilot).has_value());
    EXPECT_EQ(bus.value(throttle), 0.25);
}

TEST(ControlBus, CommandsAreClampedToTheSignalsRange) {
    ControlBus bus;
    const SignalId gimbal = bus.add_command("gimbal", "rad", {.minimum = -0.1, .maximum = 0.1}).value();
    ASSERT_TRUE(bus.publish(gimbal, ControlSource::Pilot, 5.0).has_value());
    EXPECT_EQ(bus.value(gimbal), 0.1);
    ASSERT_TRUE(bus.publish(gimbal, ControlSource::Pilot, -5.0).has_value());
    EXPECT_EQ(bus.value(gimbal), -0.1);
}

TEST(ControlBus, AFollowerTakesItsLeadersValueUnlessCommandedItself) {
    ControlBus bus;
    const SignalId all = bus.add_command("engine/throttle", "", {}).value();
    const SignalId left = bus.add_command("left/throttle", "", {}).value();
    const SignalId right = bus.add_command("right/throttle", "", {.maximum = 0.5}).value();
    ASSERT_TRUE(bus.link(all, left).has_value());
    ASSERT_TRUE(bus.link(all, right).has_value());
    ASSERT_TRUE(bus.publish(all, ControlSource::Pilot, 0.8).has_value());
    EXPECT_EQ(bus.value(left), 0.8);
    EXPECT_EQ(bus.value(right), 0.5); // within its own range
    // Shutting one engine down by its own signal leaves the other on the common throttle.
    ASSERT_TRUE(bus.publish(left, ControlSource::Pilot, 0.0).has_value());
    EXPECT_EQ(bus.value(left), 0.0);
    EXPECT_EQ(bus.value(right), 0.5);
}

TEST(ControlBus, TelemetryIsReportedNotCommanded) {
    ControlBus bus;
    const SignalId thrust = bus.add_telemetry("thrust_n", "N").value();
    const SignalId throttle = bus.add_command("throttle", "", {}).value();
    ASSERT_TRUE(bus.report(thrust, 1234.5).has_value());
    EXPECT_EQ(bus.value(thrust), 1234.5);
    EXPECT_EQ(bus.find("thrust_n").value(), thrust);
    EXPECT_EQ(bus.signals()[thrust.index].unit, "N");
    EXPECT_EQ(bus.publish(thrust, ControlSource::Pilot, 1.0).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(bus.release(thrust, ControlSource::Pilot).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(bus.report(throttle, 1.0).error().code, ErrorCode::InvalidArgument);
}

TEST(ControlBus, RejectsInvalidSignalsCommandsAndLinks) {
    ControlBus bus;
    const SignalId first = bus.add_command("a", "", {}).value();
    const SignalId second = bus.add_command("b", "", {}).value();
    EXPECT_EQ(bus.add_command("a", "", {}).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(bus.add_telemetry("", "").error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(bus.add_command("c", "", {.minimum = 1.0, .maximum = 0.0}).error().code, ErrorCode::OutOfRange);
    EXPECT_EQ(bus.add_command("c", "", {.default_value = 2.0}).error().code, ErrorCode::OutOfRange);
    EXPECT_EQ(bus.find("missing").error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(bus.publish(first, ControlSource::Pilot, std::numeric_limits<double>::quiet_NaN()).error().code,
              ErrorCode::NotFinite);
    EXPECT_EQ(bus.publish(SignalId{99}, ControlSource::Pilot, 1.0).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(bus.value(SignalId{99}), 0.0);
    ASSERT_TRUE(bus.link(first, second).has_value());
    EXPECT_EQ(bus.link(second, first).error().code, ErrorCode::InvalidArgument); // a loop
    EXPECT_EQ(bus.link(first, first).error().code, ErrorCode::InvalidArgument);
}

TEST(ControlBus, AdoptsTheCommandsOfSignalsWithTheSameName) {
    ControlBus before;
    const SignalId throttle = before.add_command("throttle", "", {}).value();
    const SignalId gone = before.add_command("dropped/ignition", "", {}).value();
    ASSERT_TRUE(before.publish(throttle, ControlSource::Autopilot, 0.7).has_value());
    ASSERT_TRUE(before.publish(gone, ControlSource::Pilot, 1.0).has_value());
    ControlBus after;
    const SignalId kept = after.add_command("throttle", "", {}).value();
    const SignalId fresh = after.add_command("new/ignition", "", {}).value();
    after.adopt_commands(before);
    EXPECT_EQ(after.value(kept), 0.7);
    EXPECT_EQ(after.value(fresh), 0.0);
    // The adopted command still belongs to the source that gave it.
    ASSERT_TRUE(after.release(kept, ControlSource::Autopilot).has_value());
    EXPECT_EQ(after.value(kept), 0.0);
}

} // namespace
