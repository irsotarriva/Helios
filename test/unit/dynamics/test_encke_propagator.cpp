#include "helios/dynamics/encke_propagator.hpp"

#include <cmath>
#include <cstdint>
#include <gtest/gtest.h>
#include <numbers>
#include <vector>

#include "../support/universes.hpp"

namespace {

using helios::bodies::Body;
using helios::bodies::BodyCatalog;
using helios::dynamics::EnckePropagator;
using helios::dynamics::GravityModel;
using helios::dynamics::propagate_cowell;
using helios::dynamics::PropagatorOptions;
using helios::dynamics::VesselState;
using helios::frames::FrameTree;
using helios::math::cross;
using helios::math::dot;
using helios::math::norm;
using helios::math::Vector3;
using helios::orbital::StateVector;
using helios::test::EarthMoonCr3bp;
using helios::time::Epoch;

constexpr double k_earth_mu_m3_s2 = 3.986004354e14;

[[nodiscard]] Epoch at(double seconds) {
    return Epoch::from_seconds(seconds).value();
}

TEST(EnckePropagator, ReducesToTheExactConicWithoutPerturbations) {
    const FrameTree tree("Earth");
    BodyCatalog catalog;
    const auto earth = catalog
                           .add(Body{.name = "Earth",
                                     .frame = FrameTree::root(),
                                     .gravitational_parameter_m3_s2 = k_earth_mu_m3_s2,
                                     .mean_radius_m = 6.371e6})
                           .value();
    const GravityModel gravity = GravityModel::make(tree, catalog, {}, Epoch{}).value();
    const StateVector initial{.position_m = {7.0e6, 0.0, 0.0}, .velocity_m_s = {0.0, 7'000.0, 2'000.0}};
    EnckePropagator propagator = EnckePropagator::make(gravity, {earth, Epoch{}, initial}, {}).value();

    for (const double elapsed_s : {1'000.0, 50'000.0, 500'000.0}) {
        const StateVector expected =
            helios::orbital::propagate_conic(initial, k_earth_mu_m3_s2, elapsed_s).value();
        const StateVector actual = propagator.state_at(at(elapsed_s)).value().state_in_domain;
        EXPECT_LT(norm(actual.position_m - expected.position_m), 1e-3) << elapsed_s;
    }
    EXPECT_EQ(propagator.statistics().rectifications, 0U);
}

// Warp invariance (BRIEFING §7.1, D14): the trajectory must not depend on how often it is sampled.
TEST(EnckePropagator, TrajectoryIsIdenticalAtEverySamplingCadence) {
    const EarthMoonCr3bp system = helios::test::make_earth_moon_cr3bp();
    const GravityModel gravity = GravityModel::make(*system.tree, *system.catalog, {}, Epoch{}).value();
    const StateVector initial{.position_m = {4.2e7, 0.0, 0.0}, .velocity_m_s = {0.0, 3'080.0, 100.0}};
    const VesselState start{system.earth, Epoch{}, initial};
    PropagatorOptions options;
    options.switch_domains = false;

    EnckePropagator slow_motion = EnckePropagator::make(gravity, start, options).value();
    EnckePropagator full_warp = EnckePropagator::make(gravity, start, options).value();
    constexpr double k_span_s = 30.0 * 86'400.0;
    constexpr double k_coarse_step_s = 86'400.0; // "1e5× warp": one sample per simulated day
    constexpr double k_fine_step_s = 60.0;       // "real time-ish": one sample per minute

    constexpr int k_fine_samples = static_cast<int>(k_span_s / k_fine_step_s);
    constexpr int k_fine_per_coarse = static_cast<int>(k_coarse_step_s / k_fine_step_s);
    for (int sample = 1; sample <= k_fine_samples; ++sample) {
        const double time_s = k_fine_step_s * sample;
        const auto fine = slow_motion.state_at(at(time_s)).value();
        if (sample % k_fine_per_coarse == 0) {
            const auto coarse = full_warp.state_at(at(time_s)).value();
            // Bit-identical, not merely close.
            ASSERT_EQ(fine.state_in_domain.position_m, coarse.state_in_domain.position_m) << time_s;
            ASSERT_EQ(fine.state_in_domain.velocity_m_s, coarse.state_in_domain.velocity_m_s) << time_s;
        }
    }
    EXPECT_EQ(slow_motion.statistics().accepted_steps, full_warp.statistics().accepted_steps);
}

TEST(EnckePropagator, ImpulseReosculatesTheConic) {
    const FrameTree tree("Earth");
    BodyCatalog catalog;
    const auto earth = catalog
                           .add(Body{.name = "Earth",
                                     .frame = FrameTree::root(),
                                     .gravitational_parameter_m3_s2 = k_earth_mu_m3_s2,
                                     .mean_radius_m = 6.371e6})
                           .value();
    const GravityModel gravity = GravityModel::make(tree, catalog, {}, Epoch{}).value();
    const StateVector initial{.position_m = {7.0e6, 0.0, 0.0}, .velocity_m_s = {0.0, 7'546.0, 0.0}};
    EnckePropagator propagator = EnckePropagator::make(gravity, {earth, Epoch{}, initial}, {}).value();
    constexpr double k_burn_s = 1'234.5;
    const Vector3 delta_v_m_s{-30.0, 100.0, 5.0};
    ASSERT_TRUE(propagator.schedule_impulse({at(k_burn_s), delta_v_m_s}).has_value());
    ASSERT_EQ(propagator.pending_impulses().size(), 1U);

    const StateVector before = helios::orbital::propagate_conic(initial, k_earth_mu_m3_s2, k_burn_s).value();
    const StateVector after_burn{.position_m = before.position_m,
                                 .velocity_m_s = before.velocity_m_s + delta_v_m_s};
    // The state *at* the impulse epoch is the post-impulse one.
    const StateVector at_burn = propagator.state_at(at(k_burn_s)).value().state_in_domain;
    EXPECT_LT(norm(at_burn.velocity_m_s - after_burn.velocity_m_s), 1e-9);
    const StateVector later = propagator.state_at(at(k_burn_s + 20'000.0)).value().state_in_domain;
    const StateVector expected =
        helios::orbital::propagate_conic(after_burn, k_earth_mu_m3_s2, 20'000.0).value();
    EXPECT_LT(norm(later.position_m - expected.position_m), 1e-3);
    EXPECT_EQ(propagator.statistics().impulses, 1U);
    EXPECT_TRUE(propagator.pending_impulses().empty());
    // The past cannot be rewritten.
    EXPECT_FALSE(propagator.schedule_impulse({at(k_burn_s + 10.0), delta_v_m_s}).has_value());
}

// An impulse scheduled "live", while the propagator is already inside the step that contains it,
// must give the same trajectory as one scheduled in advance, at any sampling cadence.
TEST(EnckePropagator, ImpulsesAreWarpInvariantAndIndependentOfWhenTheyWereScheduled) {
    const EarthMoonCr3bp system = helios::test::make_earth_moon_cr3bp();
    const GravityModel gravity = GravityModel::make(*system.tree, *system.catalog, {}, Epoch{}).value();
    const StateVector initial{.position_m = {4.2e7, 0.0, 0.0}, .velocity_m_s = {0.0, 3'080.0, 100.0}};
    const VesselState start{system.earth, Epoch{}, initial};
    const helios::dynamics::Impulse burn{at(40'000.37), {12.0, -250.0, 30.0}};
    constexpr double k_end_s = 5.0 * 86'400.0;

    EnckePropagator planned = EnckePropagator::make(gravity, start, {}).value();
    ASSERT_TRUE(planned.schedule_impulse(burn).has_value());
    EnckePropagator coarse = planned;
    EnckePropagator live = EnckePropagator::make(gravity, start, {}).value();

    std::vector<VesselState> planned_samples;
    std::vector<VesselState> live_samples;
    constexpr int k_samples = static_cast<int>(k_end_s / 60.0);
    for (int sample = 1; sample <= k_samples; ++sample) {
        const double time_s = 60.0 * sample;
        if (time_s > 40'000.37 && live.pending_impulses().empty() && live.statistics().impulses == 0) {
            ASSERT_TRUE(live.schedule_impulse(burn).has_value());
        }
        planned_samples.push_back(planned.state_at(at(time_s)).value());
        live_samples.push_back(live.state_at(at(time_s)).value());
    }
    // Rationale: the live burn was scheduled after the query at 39,960 s, so the propagator was
    // already inside (or at the end of) the step containing the burn.
    for (std::size_t index = 0; index < planned_samples.size(); ++index) {
        ASSERT_EQ(planned_samples[index].state_in_domain.position_m,
                  live_samples[index].state_in_domain.position_m)
            << index;
        ASSERT_EQ(planned_samples[index].state_in_domain.velocity_m_s,
                  live_samples[index].state_in_domain.velocity_m_s)
            << index;
    }
    const VesselState coarse_end = coarse.state_at(at(k_end_s)).value();
    EXPECT_EQ(coarse_end.state_in_domain.position_m, planned_samples.back().state_in_domain.position_m);
    EXPECT_EQ(coarse_end.state_in_domain.velocity_m_s, planned_samples.back().state_in_domain.velocity_m_s);
}

[[nodiscard]] PropagatorOptions with_analytic_regime() {
    PropagatorOptions options;
    options.analytic_perturbation_ratio = 1e-6;
    return options;
}

// A 630 km circular orbit: the Moon's tide there is ~1.5e-7 of Earth's gravity.
const StateVector k_low_orbit{.position_m = {7.0e6, 0.0, 0.0}, .velocity_m_s = {0.0, 7'546.0, 300.0}};
constexpr double k_day_s = 86'400.0;

// The analytic regime (BRIEFING §5.3): a weakly perturbed orbit is an exact conic, and a year of
// it costs no more than a day.
TEST(EnckePropagator, LowOrbitFollowsItsConicInTheAnalyticRegimeAtConstantCost) {
    const EarthMoonCr3bp system = helios::test::make_earth_moon_cr3bp();
    const GravityModel gravity = GravityModel::make(*system.tree, *system.catalog, {}, Epoch{}).value();
    const VesselState start{system.earth, Epoch{}, k_low_orbit};
    EnckePropagator analytic = EnckePropagator::make(gravity, start, with_analytic_regime()).value();
    EnckePropagator integrated = EnckePropagator::make(gravity, start, {}).value();

    constexpr double k_span_s = 12.0 * k_day_s;
    const StateVector analytic_state = analytic.state_at(at(k_span_s)).value().state_in_domain;
    const StateVector integrated_state = integrated.state_at(at(k_span_s)).value().state_in_domain;
    EXPECT_TRUE(analytic.is_analytic());
    EXPECT_FALSE(integrated.is_analytic());
    EXPECT_EQ(analytic.statistics().analytic_segments, 2U);
    EXPECT_LE(analytic.statistics().accepted_steps, 2U);
    EXPECT_GT(integrated.statistics().accepted_steps, 5'000U);

    // The regime starts at the first step boundary, so the conic is the initial one to within
    // that one step's perturbation.
    const StateVector conic =
        helios::orbital::propagate_conic(k_low_orbit, k_earth_mu_m3_s2, k_span_s).value();
    EXPECT_LT(norm(analytic_state.position_m - conic.position_m), 1'000.0);
    // What the regime gives up: the Moon's tide, which moves this orbit a little in 12 days.
    const double dropped_m = norm(analytic_state.position_m - integrated_state.position_m);
    EXPECT_GT(dropped_m, 1.0);
    EXPECT_LT(dropped_m, 50'000.0);

    // A century is as cheap as a month: no further integration steps.
    const auto later = analytic.state_at(at(36'525.0 * k_day_s));
    ASSERT_TRUE(later.has_value());
    EXPECT_LE(analytic.statistics().accepted_steps, 2U);
    EXPECT_NEAR(norm(later->state_in_domain.position_m), norm(k_low_orbit.position_m), 2.0e4);
}

TEST(EnckePropagator, StronglyPerturbedOrbitsNeverBecomeAnalytic) {
    const EarthMoonCr3bp system = helios::test::make_earth_moon_cr3bp();
    const GravityModel gravity = GravityModel::make(*system.tree, *system.catalog, {}, Epoch{}).value();
    // Geostationary radius: the lunar tide is ~3e-5 of Earth's gravity.
    const StateVector initial{.position_m = {4.2e7, 0.0, 0.0}, .velocity_m_s = {0.0, 3'080.0, 100.0}};
    const VesselState start{system.earth, Epoch{}, initial};
    EnckePropagator candidate = EnckePropagator::make(gravity, start, with_analytic_regime()).value();
    EnckePropagator integrated = EnckePropagator::make(gravity, start, {}).value();
    const auto candidate_state = candidate.state_at(at(30.0 * k_day_s)).value();
    const auto integrated_state = integrated.state_at(at(30.0 * k_day_s)).value();
    EXPECT_EQ(candidate.statistics().analytic_segments, 0U);
    // Considering the regime must not disturb the integration.
    EXPECT_EQ(candidate_state.state_in_domain.position_m, integrated_state.state_in_domain.position_m);
    EXPECT_EQ(candidate.statistics().accepted_steps, integrated.statistics().accepted_steps);
}

// D14 in the analytic regime, including a burn inside an analytic segment that is scheduled
// either in advance or while the propagator is already in that segment.
TEST(EnckePropagator, AnalyticRegimeIsWarpInvariantAcrossSegmentsAndImpulses) {
    const EarthMoonCr3bp system = helios::test::make_earth_moon_cr3bp();
    const GravityModel gravity = GravityModel::make(*system.tree, *system.catalog, {}, Epoch{}).value();
    const VesselState start{system.earth, Epoch{}, k_low_orbit};
    constexpr double k_burn_s = 12.3 * k_day_s + 0.37;
    const helios::dynamics::Impulse burn{at(k_burn_s), {0.0, 5.0, -2.0}};
    constexpr double k_fine_step_s = 60.0;
    constexpr int k_fine_per_coarse = 1'440; // one coarse sample per day
    constexpr int k_fine_samples = 25 * k_fine_per_coarse;

    EnckePropagator planned = EnckePropagator::make(gravity, start, with_analytic_regime()).value();
    ASSERT_TRUE(planned.schedule_impulse(burn).has_value());
    EnckePropagator coarse = planned;
    EnckePropagator live = EnckePropagator::make(gravity, start, with_analytic_regime()).value();
    for (int sample = 1; sample <= k_fine_samples; ++sample) {
        const double time_s = k_fine_step_s * sample;
        if (time_s > k_burn_s - 3'600.0 && live.pending_impulses().empty()
            && live.statistics().impulses == 0) {
            ASSERT_TRUE(live.is_analytic());
            ASSERT_TRUE(live.schedule_impulse(burn).has_value());
        }
        const auto planned_state = planned.state_at(at(time_s)).value();
        const auto live_state = live.state_at(at(time_s)).value();
        ASSERT_EQ(planned_state.state_in_domain.position_m, live_state.state_in_domain.position_m) << time_s;
        ASSERT_EQ(planned_state.state_in_domain.velocity_m_s, live_state.state_in_domain.velocity_m_s)
            << time_s;
        if (sample % k_fine_per_coarse == 0) {
            const auto coarse_state = coarse.state_at(at(time_s)).value();
            ASSERT_EQ(planned_state.state_in_domain.position_m, coarse_state.state_in_domain.position_m)
                << time_s;
            ASSERT_EQ(planned_state.state_in_domain.velocity_m_s, coarse_state.state_in_domain.velocity_m_s)
                << time_s;
        }
    }
    EXPECT_EQ(planned.statistics().impulses, 1U);
    EXPECT_TRUE(planned.is_analytic()) << "still a low orbit after the burn";
    EXPECT_EQ(planned.statistics().analytic_segments, coarse.statistics().analytic_segments);
    EXPECT_EQ(planned.statistics().accepted_steps, coarse.statistics().accepted_steps);
}

TEST(EnckePropagator, ABurnToAPerturbedOrbitLeavesTheAnalyticRegime) {
    const EarthMoonCr3bp system = helios::test::make_earth_moon_cr3bp();
    const GravityModel gravity = GravityModel::make(*system.tree, *system.catalog, {}, Epoch{}).value();
    EnckePropagator propagator =
        EnckePropagator::make(gravity, {system.earth, Epoch{}, k_low_orbit}, with_analytic_regime()).value();
    ASSERT_TRUE(propagator.state_at(at(2.0 * k_day_s)).has_value());
    ASSERT_TRUE(propagator.is_analytic());
    const std::uint64_t steps_before = propagator.statistics().accepted_steps;

    // Raise the apoapsis to ~55,000 km, where the lunar tide is far above the threshold.
    const StateVector at_burn = propagator.state_at(at(3.0 * k_day_s)).value().state_in_domain;
    const Vector3 prograde = at_burn.velocity_m_s / norm(at_burn.velocity_m_s);
    ASSERT_TRUE(propagator.schedule_impulse({at(3.0 * k_day_s), 2'500.0 * prograde}).has_value());
    const auto after = propagator.state_at(at(10.0 * k_day_s));
    ASSERT_TRUE(after.has_value());
    EXPECT_FALSE(propagator.is_analytic());
    EXPECT_GT(propagator.statistics().accepted_steps, steps_before + 100U);
}

TEST(EnckePropagator, AgreesWithCowellInTheRealSolarSystem) {
    const auto solar_system = helios::test::load_solar_system();
    const auto earth = solar_system.catalog->find("Earth").value();
    const Epoch start = Epoch::from_parts(631'238'400, 0.0).value();
    const GravityModel gravity =
        GravityModel::make(*solar_system.tree, *solar_system.catalog, {}, start).value();

    // A highly elliptical orbit (perigee ~7,000 km, apogee ~150,000 km): lunar and solar
    // perturbations are strong at apogee.
    const StateVector initial{.position_m = {7.0e6, 0.0, 0.0}, .velocity_m_s = {0.0, 10'200.0, 1'000.0}};
    constexpr double k_duration_s = 10.0 * 86'400.0;
    PropagatorOptions options;
    options.switch_domains = false;
    EnckePropagator encke = EnckePropagator::make(gravity, {earth, start, initial}, options).value();
    const StateVector encke_state =
        encke.state_at(start.advanced_by(k_duration_s).value()).value().state_in_domain;
    const StateVector cowell_state =
        propagate_cowell(gravity, earth, start, initial, k_duration_s, 1e-13, 600.0).value();

    // The perturbation over 10 days is hundreds of km; the two formulations agree to metres.
    const StateVector kepler_only =
        helios::orbital::propagate_conic(initial, k_earth_mu_m3_s2, k_duration_s).value();
    EXPECT_GT(norm(cowell_state.position_m - kepler_only.position_m), 1e5);
    EXPECT_LT(norm(encke_state.position_m - cowell_state.position_m), 10.0);
    EXPECT_LT(norm(encke_state.velocity_m_s - cowell_state.velocity_m_s), 1e-3);
}

TEST(EnckePropagator, SwitchesDomainWhenLeavingTheMoonsSphereOfInfluence) {
    const EarthMoonCr3bp system = helios::test::make_earth_moon_cr3bp();
    const GravityModel gravity = GravityModel::make(*system.tree, *system.catalog, {}, Epoch{}).value();
    // Start 5,000 km above the Moon moving outward at ~3 km/s: escapes the 66,000 km sphere.
    const StateVector initial{.position_m = {6.7e6, 0.0, 0.0}, .velocity_m_s = {3'000.0, 500.0, 0.0}};
    EnckePropagator propagator = EnckePropagator::make(gravity, {system.moon, Epoch{}, initial}, {}).value();
    const auto after = propagator.state_at(at(2.0 * 86'400.0)).value();
    EXPECT_EQ(after.domain, system.earth);
    EXPECT_EQ(propagator.statistics().domain_changes, 1U);

    // The same trajectory integrated by Cowell entirely in the Earth frame.
    const auto moon_in_earth = system.tree
                                   ->relative_state(system.catalog->body(system.moon)->get().frame,
                                                    system.catalog->body(system.earth)->get().frame, Epoch{})
                                   .value();
    const StateVector initial_in_earth{.position_m = initial.position_m + moon_in_earth.position_m,
                                       .velocity_m_s = initial.velocity_m_s + moon_in_earth.velocity_m_s};
    const StateVector reference =
        propagate_cowell(gravity, system.earth, Epoch{}, initial_in_earth, 2.0 * 86'400.0, 1e-13, 60.0)
            .value();
    EXPECT_LT(norm(after.state_in_domain.position_m - reference.position_m), 10.0);
}

// Barycentric position/velocity of a vessel propagated in the Earth domain.
[[nodiscard]] StateVector barycentric(const EarthMoonCr3bp& system, const VesselState& state) {
    const auto earth =
        system.tree
            ->relative_state(system.catalog->body(system.earth)->get().frame, FrameTree::root(), state.epoch)
            .value();
    return StateVector{.position_m = state.state_in_domain.position_m + earth.position_m,
                       .velocity_m_s = state.state_in_domain.velocity_m_s + earth.velocity_m_s};
}

[[nodiscard]] helios::frames::FrameId frame_of(const EarthMoonCr3bp& system, helios::bodies::BodyId body) {
    return system.catalog->body(body)->get().frame;
}

// Jacobi integral of the circular restricted three-body problem, from barycentric inertial
// quantities: C = ½|V|² − n·(R × V)_z − μ_E/r_E − μ_M/r_M. Exactly conserved by the true dynamics.
[[nodiscard]] double jacobi_constant(const EarthMoonCr3bp& system, const StateVector& vessel,
                                     const Epoch& instant) {
    const Vector3 earth_m =
        system.tree->relative_state(frame_of(system, system.earth), FrameTree::root(), instant)
            .value()
            .position_m;
    const Vector3 moon_m =
        system.tree->relative_state(frame_of(system, system.moon), FrameTree::root(), instant)
            .value()
            .position_m;
    const double angular_momentum_z_m2_s = cross(vessel.position_m, vessel.velocity_m_s).z;
    return 0.5 * dot(vessel.velocity_m_s, vessel.velocity_m_s)
           - EarthMoonCr3bp::mean_motion_rad_s() * angular_momentum_z_m2_s
           - EarthMoonCr3bp::k_earth_mu_m3_s2 / norm(vessel.position_m - earth_m)
           - EarthMoonCr3bp::k_moon_mu_m3_s2 / norm(vessel.position_m - moon_m);
}

// Barycentric position rotated into the frame co-rotating with the Moon (Moon on +x).
[[nodiscard]] Vector3 co_rotating(const Vector3& position_m, double time_s) {
    const double angle_rad = -EarthMoonCr3bp::mean_motion_rad_s() * time_s;
    return {position_m.x * std::cos(angle_rad) - position_m.y * std::sin(angle_rad),
            position_m.x * std::sin(angle_rad) + position_m.y * std::cos(angle_rad), position_m.z};
}

// Vessel state (in the Earth domain) at rest in the co-rotating frame at barycentric `point_m`.
[[nodiscard]] StateVector co_rotating_rest_state(const EarthMoonCr3bp& system, const Vector3& point_m) {
    const double mean_motion_rad_s = EarthMoonCr3bp::mean_motion_rad_s();
    const Vector3 velocity_m_s{-mean_motion_rad_s * point_m.y, mean_motion_rad_s * point_m.x, 0.0};
    const auto earth =
        system.tree->relative_state(frame_of(system, system.earth), FrameTree::root(), Epoch{}).value();
    return StateVector{.position_m = point_m - earth.position_m,
                       .velocity_m_s = velocity_m_s - earth.velocity_m_s};
}

TEST(EnckePropagator, TrojanPointL4IsStableAndTheJacobiConstantIsConserved) {
    const EarthMoonCr3bp system = helios::test::make_earth_moon_cr3bp();
    const GravityModel gravity = GravityModel::make(*system.tree, *system.catalog, {}, Epoch{}).value();
    const double separation_m = EarthMoonCr3bp::k_separation_m;
    const double mass_ratio = EarthMoonCr3bp::mass_ratio();
    // L4: equilateral with Earth and Moon, 60° ahead of the Moon. Displace it by 1,000 km.
    const Vector3 l4_m{separation_m * (0.5 - mass_ratio), separation_m * std::numbers::sqrt3 / 2.0, 0.0};
    const Vector3 start_m = l4_m + Vector3{1.0e6, 0.0, 0.0};

    PropagatorOptions options;
    options.switch_domains = false;
    EnckePropagator propagator =
        EnckePropagator::make(gravity, {system.earth, Epoch{}, co_rotating_rest_state(system, start_m)},
                              options)
            .value();
    const double initial_jacobi = jacobi_constant(
        system, barycentric(system, {system.earth, Epoch{}, co_rotating_rest_state(system, start_m)}),
        Epoch{});
    double worst_excursion_m = 0.0;
    double worst_jacobi_drift = 0.0;
    for (int day = 0; day <= 365; ++day) {
        const double time_s = 86'400.0 * day;
        const auto state = propagator.state_at(at(time_s)).value();
        const StateVector vessel = barycentric(system, state);
        worst_excursion_m = std::max(worst_excursion_m, norm(co_rotating(vessel.position_m, time_s) - l4_m));
        worst_jacobi_drift = std::max(
            worst_jacobi_drift, std::abs(jacobi_constant(system, vessel, at(time_s)) / initial_jacobi - 1.0));
    }
    // Bounded libration around L4 for a year, and the integral conserved to ~1e-10.
    EXPECT_LT(worst_excursion_m, 0.05 * separation_m);
    EXPECT_LT(worst_jacobi_drift, 1e-9);
}

// Collinear point L1 of the CR3BP: distance γ from the Moon (units of the separation) solves
// γ⁵ − (3 − μ)γ⁴ + (3 − 2μ)γ³ − μγ² + 2μγ − μ = 0 (Szebehely 1967).
[[nodiscard]] double l1_gamma(double mass_ratio) {
    double gamma = std::cbrt(mass_ratio / 3.0);
    for (int iteration = 0; iteration < 50; ++iteration) {
        const double value = std::pow(gamma, 5) - (3.0 - mass_ratio) * std::pow(gamma, 4)
                             + (3.0 - 2.0 * mass_ratio) * std::pow(gamma, 3) - mass_ratio * gamma * gamma
                             + 2.0 * mass_ratio * gamma - mass_ratio;
        const double slope = 5.0 * std::pow(gamma, 4) - 4.0 * (3.0 - mass_ratio) * std::pow(gamma, 3)
                             + 3.0 * (3.0 - 2.0 * mass_ratio) * gamma * gamma - 2.0 * mass_ratio * gamma
                             + 2.0 * mass_ratio;
        gamma -= value / slope;
    }
    return gamma;
}

// The instability at L1 is physical. Its growth rate must be the linearised CR3BP eigenvalue, not
// something set by the integrator: numerical error only seeds it (BRIEFING §7.1).
TEST(EnckePropagator, InstabilityAtL1GrowsAtTheLinearTheoryRate) {
    const EarthMoonCr3bp system = helios::test::make_earth_moon_cr3bp();
    const GravityModel gravity = GravityModel::make(*system.tree, *system.catalog, {}, Epoch{}).value();
    const double mass_ratio = EarthMoonCr3bp::mass_ratio();
    const double gamma = l1_gamma(mass_ratio);
    const double separation_m = EarthMoonCr3bp::k_separation_m;
    const Vector3 l1_m{(1.0 - mass_ratio - gamma) * separation_m, 0.0, 0.0};

    // Linear theory: λ² = (c₂ − 2 + √(9c₂² − 8c₂))/2 with c₂ = μ/γ³ + (1 − μ)/(1 − γ)³.
    const double c2 = mass_ratio / std::pow(gamma, 3) + (1.0 - mass_ratio) / std::pow(1.0 - gamma, 3);
    const double expected_rate_per_s = std::sqrt((c2 - 2.0 + std::sqrt(9.0 * c2 * c2 - 8.0 * c2)) / 2.0)
                                       * EarthMoonCr3bp::mean_motion_rad_s();

    PropagatorOptions options;
    options.switch_domains = false;
    const Vector3 start_m = l1_m + Vector3{10.0, 0.0, 0.0};
    EnckePropagator propagator =
        EnckePropagator::make(gravity, {system.earth, Epoch{}, co_rotating_rest_state(system, start_m)},
                              options)
            .value();

    // Fit ln(distance from L1) against time over the linear regime (10 km .. 10,000 km).
    std::vector<double> times_s;
    std::vector<double> log_distances;
    for (int hour = 0; hour <= 40 * 24; ++hour) {
        const double time_s = 3'600.0 * hour;
        const StateVector vessel = barycentric(system, propagator.state_at(at(time_s)).value());
        const double distance_m = norm(co_rotating(vessel.position_m, time_s) - l1_m);
        if (distance_m > 1.0e4 && distance_m < 1.0e7) {
            times_s.push_back(time_s);
            log_distances.push_back(std::log(distance_m));
        }
    }
    ASSERT_GT(times_s.size(), 20U);
    double mean_time = 0.0;
    double mean_log = 0.0;
    for (std::size_t index = 0; index < times_s.size(); ++index) {
        mean_time += times_s[index];
        mean_log += log_distances[index];
    }
    mean_time /= static_cast<double>(times_s.size());
    mean_log /= static_cast<double>(times_s.size());
    double covariance = 0.0;
    double variance = 0.0;
    for (std::size_t index = 0; index < times_s.size(); ++index) {
        covariance += (times_s[index] - mean_time) * (log_distances[index] - mean_log);
        variance += (times_s[index] - mean_time) * (times_s[index] - mean_time);
    }
    const double measured_rate_per_s = covariance / variance;
    EXPECT_NEAR(measured_rate_per_s / expected_rate_per_s, 1.0, 0.03)
        << "measured e-folding time " << 1.0 / measured_rate_per_s / 86'400.0 << " d, expected "
        << 1.0 / expected_rate_per_s / 86'400.0 << " d";
}

} // namespace
