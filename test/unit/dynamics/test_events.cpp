#include "helios/dynamics/events.hpp"
#include "helios/dynamics/trajectory.hpp"
#include "helios/orbital/conic.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <optional>

#include "../support/universes.hpp"

namespace {

using helios::bodies::Body;
using helios::bodies::BodyCatalog;
using helios::bodies::BodyId;
using helios::dynamics::EnckePropagator;
using helios::dynamics::EventKind;
using helios::dynamics::EventSearchOptions;
using helios::dynamics::GravityModel;
using helios::dynamics::predict_conic_events;
using helios::dynamics::predict_trajectory;
using helios::dynamics::PredictedEvent;
using helios::dynamics::PredictionOptions;
using helios::dynamics::VesselState;
using helios::frames::FrameTree;
using helios::math::norm;
using helios::orbital::StateVector;
using helios::test::EarthMoonCr3bp;
using helios::time::Epoch;
using helios::time::seconds_between;

constexpr double k_earth_mu_m3_s2 = 3.986004354e14;
constexpr double k_day_s = 86'400.0;

[[nodiscard]] Epoch at(double seconds) {
    return Epoch::from_seconds(seconds).value();
}

[[nodiscard]] std::optional<PredictedEvent> first_of(const std::vector<PredictedEvent>& events,
                                                     EventKind kind) {
    for (const PredictedEvent& event : events) {
        if (event.kind == kind) {
            return event;
        }
    }
    return std::nullopt;
}

// First sample time (60 s cadence) at which the propagator reports a different domain.
[[nodiscard]] std::optional<double> observed_switch_s(EnckePropagator propagator, BodyId from,
                                                      double span_s) {
    const int samples = static_cast<int>(span_s / 60.0);
    for (int sample = 1; sample <= samples; ++sample) {
        const double time_s = 60.0 * sample;
        if (propagator.state_at(at(time_s)).value().domain != from) {
            return time_s;
        }
    }
    return std::nullopt;
}

struct EarthOnly {
    FrameTree tree{"Earth"};
    BodyCatalog catalog;
    BodyId earth;
};

TEST(PredictConicEvents, SuborbitalArcReportsApoapsisThenImpactAndNothingAfter) {
    EarthOnly universe;
    universe.earth = universe.catalog
                         .add(Body{.name = "Earth",
                                   .frame = FrameTree::root(),
                                   .gravitational_parameter_m3_s2 = k_earth_mu_m3_s2,
                                   .mean_radius_m = 6.371e6})
                         .value();
    const GravityModel gravity = GravityModel::make(universe.tree, universe.catalog, {}, Epoch{}).value();
    // 100 km up, climbing steeply at 3 km/s: a sounding-rocket arc.
    const VesselState vessel{
        universe.earth, Epoch{},
        StateVector{.position_m = {6.471e6, 0.0, 0.0}, .velocity_m_s = {2'500.0, 1'500.0, 0.0}}};
    const auto events =
        predict_conic_events(gravity, vessel, EventSearchOptions{.horizon_s = k_day_s}).value();
    ASSERT_EQ(events.size(), 2U);
    EXPECT_EQ(events[0].kind, EventKind::Apoapsis);
    EXPECT_EQ(events[1].kind, EventKind::SurfaceImpact);
    const auto at_impact = helios::orbital::propagate_conic(vessel.state_in_domain, k_earth_mu_m3_s2,
                                                            seconds_between(Epoch{}, events[1].epoch))
                               .value();
    EXPECT_NEAR(norm(at_impact.position_m), 6.371e6, 1e-3);
}

TEST(PredictConicEvents, PredictsLeavingTheMoonsSphereCloseToTheActualSwitch) {
    const EarthMoonCr3bp system = helios::test::make_earth_moon_cr3bp();
    const GravityModel gravity = GravityModel::make(*system.tree, *system.catalog, {}, Epoch{}).value();
    const VesselState vessel{
        system.moon, Epoch{},
        StateVector{.position_m = {6.7e6, 0.0, 0.0}, .velocity_m_s = {3'000.0, 500.0, 0.0}}};
    const auto events =
        predict_conic_events(gravity, vessel, EventSearchOptions{.horizon_s = 3.0 * k_day_s}).value();
    const auto exit = first_of(events, EventKind::DomainExit);
    ASSERT_TRUE(exit.has_value());
    EXPECT_EQ(events.back().kind, EventKind::DomainExit) << "nothing is reported after leaving the domain";

    const double predicted_s = seconds_between(Epoch{}, exit->epoch);
    const auto actual_s =
        observed_switch_s(EnckePropagator::make(gravity, vessel, {}).value(), system.moon, 3.0 * k_day_s);
    ASSERT_TRUE(actual_s.has_value());
    // The conic ignores Earth's pull; the prediction is a warp-limiting schedule, good to ~1%.
    EXPECT_NEAR(*actual_s / predicted_s, 1.0, 0.02);
}

TEST(PredictConicEvents, FindsEntryIntoTheMoonsSphereFromAnEarthOrbit) {
    const EarthMoonCr3bp system = helios::test::make_earth_moon_cr3bp();
    const GravityModel gravity = GravityModel::make(*system.tree, *system.catalog, {}, Epoch{}).value();
    const BodyId earth = system.earth;
    const auto& moon_body = system.catalog->body(system.moon)->get();
    const auto& earth_body = system.catalog->body(earth)->get();

    // Build a state that is 40,000 km from the Moon, falling towards it, two days from now; then
    // run the Earth conic back two days to get an Earth-domain start that will enter the sphere.
    constexpr double k_arrival_s = 2.0 * k_day_s;
    const auto moon_at_arrival =
        system.tree->relative_state(moon_body.frame, earth_body.frame, at(k_arrival_s)).value();
    const StateVector arrival{
        .position_m = moon_at_arrival.position_m + helios::math::Vector3{-4.0e7, 0.0, 0.0},
        .velocity_m_s = moon_at_arrival.velocity_m_s + helios::math::Vector3{1'200.0, 300.0, 0.0}};
    const StateVector start =
        helios::orbital::propagate_conic(arrival, k_earth_mu_m3_s2, -k_arrival_s).value();
    const VesselState vessel{earth, Epoch{}, start};

    const auto events =
        predict_conic_events(gravity, vessel, EventSearchOptions{.horizon_s = 4.0 * k_day_s}).value();
    const auto entry = first_of(events, EventKind::DomainEntry);
    ASSERT_TRUE(entry.has_value());
    EXPECT_EQ(entry->body, system.moon);
    const double predicted_s = seconds_between(Epoch{}, entry->epoch);
    EXPECT_GT(predicted_s, 0.0);
    EXPECT_LT(predicted_s, k_arrival_s);

    // On the conic, the vessel is exactly on the entry sphere at the predicted instant.
    const StateVector on_conic =
        helios::orbital::propagate_conic(start, k_earth_mu_m3_s2, predicted_s).value();
    const auto moon_then =
        system.tree->relative_state(moon_body.frame, earth_body.frame, entry->epoch).value();
    EXPECT_NEAR(norm(on_conic.position_m - moon_then.position_m) / (moon_body.domain_radius_m * 0.95), 1.0,
                1e-6);

    const auto actual_s =
        observed_switch_s(EnckePropagator::make(gravity, vessel, {}).value(), earth, 4.0 * k_day_s);
    ASSERT_TRUE(actual_s.has_value());
    EXPECT_NEAR(*actual_s / predicted_s, 1.0, 0.05);
}

TEST(PredictTrajectory, SamplesAreExactlyThePropagatorsFutureIncludingImpulsesAndDomainChanges) {
    const EarthMoonCr3bp system = helios::test::make_earth_moon_cr3bp();
    const GravityModel gravity = GravityModel::make(*system.tree, *system.catalog, {}, Epoch{}).value();
    const VesselState vessel{
        system.moon, Epoch{},
        StateVector{.position_m = {6.7e6, 0.0, 0.0}, .velocity_m_s = {3'000.0, 500.0, 0.0}}};
    EnckePropagator propagator = EnckePropagator::make(gravity, vessel, {}).value();
    ASSERT_TRUE(propagator.schedule_impulse({at(3'600.0), {0.0, 150.0, 20.0}}).has_value());
    const auto before = propagator.statistics();

    const auto segments =
        predict_trajectory(propagator, Epoch{}, PredictionOptions{.horizon_s = 3.0 * k_day_s}).value();
    // The caller's propagator is untouched.
    EXPECT_EQ(propagator.statistics().accepted_steps, before.accepted_steps);
    ASSERT_EQ(segments.size(), 2U);
    EXPECT_EQ(segments[0].domain, system.moon);
    EXPECT_EQ(segments[1].domain, system.earth);

    for (const auto& segment : segments) {
        for (const auto& sample : segment.samples) {
            const VesselState actual = propagator.state_at(sample.epoch).value();
            ASSERT_EQ(actual.domain, segment.domain);
            ASSERT_EQ(actual.state_in_domain.position_m, sample.position_m);
        }
    }
    EXPECT_EQ(seconds_between(Epoch{}, segments.back().samples.back().epoch), 3.0 * k_day_s);
}

TEST(PredictTrajectory, StopsAtTheSurface) {
    EarthOnly universe;
    universe.earth = universe.catalog
                         .add(Body{.name = "Earth",
                                   .frame = FrameTree::root(),
                                   .gravitational_parameter_m3_s2 = k_earth_mu_m3_s2,
                                   .mean_radius_m = 6.371e6})
                         .value();
    const GravityModel gravity = GravityModel::make(universe.tree, universe.catalog, {}, Epoch{}).value();
    const VesselState vessel{
        universe.earth, Epoch{},
        StateVector{.position_m = {6.471e6, 0.0, 0.0}, .velocity_m_s = {2'500.0, 1'500.0, 0.0}}};
    const auto segments = predict_trajectory(EnckePropagator::make(gravity, vessel, {}).value(), Epoch{},
                                             PredictionOptions{.horizon_s = k_day_s})
                              .value();
    ASSERT_EQ(segments.size(), 1U);
    EXPECT_LE(norm(segments[0].samples.back().position_m), 6.371e6);
    EXPECT_LT(seconds_between(Epoch{}, segments[0].samples.back().epoch), 3'600.0);
}

} // namespace
