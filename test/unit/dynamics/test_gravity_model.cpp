#include "helios/dynamics/gravity_model.hpp"

#include <algorithm>
#include <gtest/gtest.h>

#include "../support/universes.hpp"

namespace {

using helios::bodies::Body;
using helios::bodies::BodyCatalog;
using helios::dynamics::GravityModel;
using helios::frames::FixedOffset;
using helios::frames::FrameTree;
using helios::math::norm;
using helios::math::Vector3;
using helios::time::Epoch;

// Static test system: a star, a planet 1e11 m away and its moon 4e8 m beyond it.
struct StaticSystem {
    FrameTree tree{"Barycentre"};
    BodyCatalog catalog;
    helios::bodies::BodyId star;
    helios::bodies::BodyId planet;

    StaticSystem() {
        const auto star_frame =
            tree.add_frame("Star", FrameTree::root(), FixedOffset{{0.0, 0.0, 0.0}}).value();
        const auto planet_frame =
            tree.add_frame("Planet", FrameTree::root(), FixedOffset{{1e11, 0.0, 0.0}}).value();
        const auto moon_frame = tree.add_frame("Moon", planet_frame, FixedOffset{{4e8, 0.0, 0.0}}).value();
        star = catalog
                   .add(Body{.name = "Star",
                             .frame = star_frame,
                             .gravitational_parameter_m3_s2 = 1e20,
                             .mean_radius_m = 7e8})
                   .value();
        planet = catalog
                     .add(Body{.name = "Planet",
                               .frame = planet_frame,
                               .gravitational_parameter_m3_s2 = 4e14,
                               .mean_radius_m = 6.4e6,
                               .domain_parent = star,
                               .domain_radius_m = 9e8})
                     .value();
        static_cast<void>(catalog
                              .add(Body{.name = "Moon",
                                        .frame = moon_frame,
                                        .gravitational_parameter_m3_s2 = 5e12,
                                        .mean_radius_m = 1.7e6,
                                        .domain_parent = planet,
                                        .domain_radius_m = 6e7})
                              .value());
    }
};

TEST(GravityModel, SumsTheSelectedPointMassesExceptTheAnchor) {
    const StaticSystem system;
    const GravityModel gravity = GravityModel::make(system.tree, system.catalog, {}, Epoch{}).value();
    const Vector3 vessel_m{7e6, 0.0, 0.0};
    const Vector3 perturbation_m_s2 =
        gravity.perturbing_acceleration_m_s2(system.planet, vessel_m, Epoch{}).value();

    const auto pull = [&](const Vector3& source_m, double mu_m3_s2) {
        const Vector3 separation_m = source_m - vessel_m;
        return separation_m * (mu_m3_s2 / std::pow(norm(separation_m), 3));
    };
    // Frames are fixed, so the anchor's own acceleration (the indirect term) is zero here.
    const Vector3 expected_m_s2 = pull({-1e11, 0.0, 0.0}, 1e20) + pull({4e8, 0.0, 0.0}, 5e12);
    EXPECT_LT(norm(perturbation_m_s2 - expected_m_s2), 1e-15 * norm(expected_m_s2));

    const auto sources = gravity.selected_sources(system.planet, vessel_m, Epoch{}).value();
    EXPECT_EQ(sources.size(), 2U);
}

TEST(GravityModel, DistantSubsystemsCollapseToTheirBarycentre) {
    const StaticSystem system;
    // Seen from near the star, the planet+moon system (extent 5e8 m at 1e11 m) is one source...
    const GravityModel coarse =
        GravityModel::make(system.tree, system.catalog, {.opening_angle = 0.5}, Epoch{}).value();
    const auto coarse_sources = coarse.selected_sources(system.star, {1e9, 0.0, 0.0}, Epoch{}).value();
    ASSERT_EQ(coarse_sources.size(), 1U);
    EXPECT_TRUE(coarse_sources[0].is_aggregate);
    EXPECT_DOUBLE_EQ(coarse_sources[0].gravitational_parameter_m3_s2, 4e14 + 5e12);

    // ...and with θ = 0 it is always opened. The difference is the tiny quadrupole term.
    const GravityModel exact =
        GravityModel::make(system.tree, system.catalog, {.opening_angle = 0.0}, Epoch{}).value();
    EXPECT_EQ(exact.selected_sources(system.star, {1e9, 0.0, 0.0}, Epoch{}).value().size(), 2U);
    const Vector3 coarse_m_s2 =
        coarse.perturbing_acceleration_m_s2(system.star, {1e9, 0.0, 0.0}, Epoch{}).value();
    const Vector3 exact_m_s2 =
        exact.perturbing_acceleration_m_s2(system.star, {1e9, 0.0, 0.0}, Epoch{}).value();
    EXPECT_LT(norm(coarse_m_s2 - exact_m_s2) / norm(exact_m_s2), 1e-4);
}

TEST(GravityModel, LowEarthOrbitSeesOnlyTidalPerturbationsInTheRealSolarSystem) {
    const auto solar_system = helios::test::load_solar_system();
    const auto earth = solar_system.catalog->find("Earth").value();
    const GravityModel gravity = GravityModel::make(*solar_system.tree, *solar_system.catalog, {},
                                                    Epoch::from_parts(631'238'400, 0.0).value())
                                     .value();
    const Epoch instant = Epoch::from_parts(631'238'400, 0.0).value();
    const Vector3 perturbation_m_s2 =
        gravity.perturbing_acceleration_m_s2(earth, {7e6, 0.0, 0.0}, instant).value();
    // The Sun pulls with ~5.9e-3 m/s², but Earth falls with it: only the ~1e-6 tide remains.
    EXPECT_GT(norm(perturbation_m_s2), 1e-7);
    EXPECT_LT(norm(perturbation_m_s2), 5e-6);

    const auto sources = gravity.selected_sources(earth, {7e6, 0.0, 0.0}, instant).value();
    const auto has_source = [&](std::string_view frame_name) {
        const auto frame = solar_system.tree->find(frame_name).value();
        return std::ranges::any_of(sources, [&](const auto& source) { return source.frame == frame; });
    };
    EXPECT_TRUE(has_source("Moon"));
    EXPECT_TRUE(has_source("Sun"));
    EXPECT_TRUE(has_source("Jupiter barycentre"));
    EXPECT_FALSE(has_source("Earth"));
}

TEST(GravityModel, RejectsNegativeOpeningAngle) {
    const StaticSystem system;
    EXPECT_FALSE(
        GravityModel::make(system.tree, system.catalog, {.opening_angle = -1.0}, Epoch{}).has_value());
}

} // namespace
