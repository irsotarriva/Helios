#include "helios/frames/frame_tree.hpp"

#include <gtest/gtest.h>

namespace {

using helios::core::ErrorCode;
using helios::ephemeris::KeplerianEphemeris;
using helios::ephemeris::SecularElements;
using helios::frames::FixedOffset;
using helios::frames::FrameId;
using helios::frames::FrameTree;
using helios::math::norm;
using helios::math::Vector3;
using helios::time::Epoch;

TEST(FrameTree, ComposesOffsetsThroughTheCommonAncestor) {
    FrameTree tree("root");
    const FrameId a = tree.add_frame("A", FrameTree::root(), FixedOffset{{1.0, 0.0, 0.0}}).value();
    const FrameId b = tree.add_frame("B", a, FixedOffset{{0.0, 2.0, 0.0}}).value();
    const FrameId c = tree.add_frame("C", FrameTree::root(), FixedOffset{{0.0, 0.0, 5.0}}).value();

    EXPECT_EQ(tree.lowest_common_ancestor(b, c).value(), FrameTree::root());
    EXPECT_EQ(tree.lowest_common_ancestor(b, a).value(), a);
    EXPECT_EQ(tree.relative_state(b, c, Epoch{}).value().position_m, (Vector3{1.0, 2.0, -5.0}));
    EXPECT_EQ(tree.relative_state(c, b, Epoch{}).value().position_m, (Vector3{-1.0, -2.0, 5.0}));
    EXPECT_EQ(tree.relative_state(b, b, Epoch{}).value().position_m, (Vector3{}));
    EXPECT_EQ(tree.depth(b).value(), 2U);
    EXPECT_EQ(tree.find("B").value(), b);
    EXPECT_EQ(tree.name(c).value(), "C");
}

TEST(FrameTree, RejectsInvalidStructure) {
    FrameTree tree("root");
    EXPECT_EQ(tree.add_frame("X", FrameId{7}, FixedOffset{}).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(tree.add_frame("", FrameTree::root(), FixedOffset{}).error().code, ErrorCode::InvalidArgument);
    ASSERT_TRUE(tree.add_frame("X", FrameTree::root(), FixedOffset{}).has_value());
    EXPECT_EQ(tree.add_frame("X", FrameTree::root(), FixedOffset{}).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(tree.parent(FrameTree::root()).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(tree.find("nope").error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(tree.relative_state(FrameId{99}, FrameTree::root(), Epoch{}).error().code,
              ErrorCode::InvalidArgument);
}

TEST(FrameTree, NearbyFramesStayExactFarFromTheRoot) {
    // A star system 8 kpc from the galactic root: a double there has a 32 km ULP.
    constexpr double k_galactic_offset_m = 2.5e20;
    constexpr double k_earth_moon_distance_m = 384'400'000.123456;
    FrameTree tree("Galactic centre");
    const FrameId sun =
        tree.add_frame("Sun", FrameTree::root(), FixedOffset{{k_galactic_offset_m, 0.0, 0.0}}).value();
    const FrameId earth = tree.add_frame("Earth", sun, FixedOffset{{1.496e11, 0.0, 0.0}}).value();
    const FrameId moon =
        tree.add_frame("Moon", earth, FixedOffset{{k_earth_moon_distance_m, 0.0, 0.0}}).value();

    const double via_common_ancestor_m = tree.relative_state(moon, earth, Epoch{}).value().position_m.x;
    EXPECT_EQ(via_common_ancestor_m, k_earth_moon_distance_m);

    // What a flat, root-relative scheme would compute instead.
    const double moon_from_root_m =
        tree.relative_state(moon, FrameTree::root(), Epoch{}).value().position_m.x;
    const double earth_from_root_m =
        tree.relative_state(earth, FrameTree::root(), Epoch{}).value().position_m.x;
    EXPECT_GT(std::abs((moon_from_root_m - earth_from_root_m) - k_earth_moon_distance_m), 1'000.0);
}

TEST(FrameTree, RelativeAccelerationIncludesTheObserversIndirectTerm) {
    FrameTree tree("Sun");
    const SecularElements orbit{
        .semi_major_axis_m = 1.496e11, .eccentricity = 0.0167, .mean_longitude_rate_rad_s = 1.99e-7};
    const FrameId earth =
        tree.add_frame("Earth", FrameTree::root(), KeplerianEphemeris::make(orbit).value()).value();
    const FrameId probe = tree.add_frame("Probe", FrameTree::root(), FixedOffset{{2e11, 0.0, 0.0}}).value();

    const Epoch instant = Epoch::from_seconds(1e6).value();
    const Vector3 earth_acceleration_m_s2 =
        tree.relative_acceleration_m_s2(earth, FrameTree::root(), instant).value();
    const Vector3 probe_seen_from_earth_m_s2 = tree.relative_acceleration_m_s2(probe, earth, instant).value();
    EXPECT_LT(norm(probe_seen_from_earth_m_s2 + earth_acceleration_m_s2), 1e-18);
    EXPECT_GT(norm(earth_acceleration_m_s2), 5e-3); // ≈ 5.9 mm/s² at 1 AU
}

} // namespace
