#ifndef HELIOS_TOOLS_SUPPORT_UNIVERSES_HPP
#define HELIOS_TOOLS_SUPPORT_UNIVERSES_HPP

// Universes shared by tests and developer tools. Not part of the Helios API.

#include "helios/bodies/body_catalog.hpp"
#include "helios/dynamics/gravity_model.hpp"
#include "helios/ephemeris/ephemeris_file.hpp"
#include "helios/frames/body_rotation.hpp"
#include "helios/frames/frame_tree.hpp"

#include <cmath>
#include <filesystem>
#include <memory>

namespace helios::tools {

// The real Solar System from a Helios Chebyshev file and a bodies CSV.
struct SolarSystem {
    std::unique_ptr<frames::FrameTree> tree;
    std::unique_ptr<bodies::BodyCatalog> catalog;

    static constexpr std::int64_t k_excerpt_start_s = 631'152'000; // 2020-01-01 TDB, ≈ start of coverage
};

[[nodiscard]] inline core::Result<SolarSystem> load_solar_system(const std::filesystem::path& ephemeris_path,
                                                                 const std::filesystem::path& bodies_path) {
    auto segments = ephemeris::load_chebyshev_file(ephemeris_path);
    if (!segments) {
        return std::unexpected(segments.error());
    }
    auto tree = std::make_unique<frames::FrameTree>(segments->front().center_name);
    for (auto& segment : *segments) {
        auto added = tree->find(segment.center_name).and_then([&](frames::FrameId center) {
            return tree->add_frame(segment.target_name, center, std::move(segment.ephemeris));
        });
        if (!added) {
            return std::unexpected(added.error());
        }
    }
    auto catalog = bodies::load_bodies_csv(bodies_path, *tree, frames::icrf_to_j2000_ecliptic());
    if (!catalog) {
        return std::unexpected(catalog.error());
    }
    return SolarSystem{.tree = std::move(tree),
                       .catalog = std::make_unique<bodies::BodyCatalog>(std::move(*catalog))};
}

// Circular restricted three-body problem realised with the normal machinery: Earth and Moon on
// circular Keplerian orbits about their barycentre (the inertial root), in the xy-plane.
struct EarthMoonCr3bp {
    static constexpr double k_earth_mu_m3_s2 = 3.986004354e14;
    static constexpr double k_moon_mu_m3_s2 = 4.902800066e12;
    static constexpr double k_separation_m = 3.844e8;

    std::unique_ptr<frames::FrameTree> tree;
    std::unique_ptr<bodies::BodyCatalog> catalog;
    bodies::BodyId earth;
    bodies::BodyId moon;

    [[nodiscard]] static double total_mu_m3_s2() { return k_earth_mu_m3_s2 + k_moon_mu_m3_s2; }
    [[nodiscard]] static double mass_ratio() { return k_moon_mu_m3_s2 / total_mu_m3_s2(); }
    [[nodiscard]] static double mean_motion_rad_s() {
        return std::sqrt(total_mu_m3_s2() / (k_separation_m * k_separation_m * k_separation_m));
    }
};

[[nodiscard]] inline EarthMoonCr3bp make_earth_moon_cr3bp() {
    using ephemeris::KeplerianEphemeris;
    using ephemeris::SecularElements;
    const double mean_motion_rad_s = EarthMoonCr3bp::mean_motion_rad_s();
    auto tree = std::make_unique<frames::FrameTree>("Earth-Moon barycentre");
    const auto earth_frame =
        tree->add_frame("Earth", frames::FrameTree::root(),
                        KeplerianEphemeris::make(
                            SecularElements{.semi_major_axis_m =
                                                EarthMoonCr3bp::mass_ratio() * EarthMoonCr3bp::k_separation_m,
                                            .mean_longitude_rad = orbital::k_pi,
                                            .mean_longitude_rate_rad_s = mean_motion_rad_s})
                            .value())
            .value();
    const auto moon_frame =
        tree->add_frame("Moon", frames::FrameTree::root(),
                        KeplerianEphemeris::make(
                            SecularElements{.semi_major_axis_m = (1.0 - EarthMoonCr3bp::mass_ratio())
                                                                 * EarthMoonCr3bp::k_separation_m,
                                            .mean_longitude_rate_rad_s = mean_motion_rad_s})
                            .value())
            .value();
    auto catalog = std::make_unique<bodies::BodyCatalog>();
    const auto earth =
        catalog
            ->add(bodies::Body{.name = "Earth",
                               .frame = earth_frame,
                               .gravitational_parameter_m3_s2 = EarthMoonCr3bp::k_earth_mu_m3_s2,
                               .mean_radius_m = 6.371e6})
            .value();
    const auto moon = catalog
                          ->add(bodies::Body{.name = "Moon",
                                             .frame = moon_frame,
                                             .gravitational_parameter_m3_s2 = EarthMoonCr3bp::k_moon_mu_m3_s2,
                                             .mean_radius_m = 1.7374e6,
                                             .domain_parent = earth,
                                             .domain_radius_m = 6.6e7})
                          .value();
    return EarthMoonCr3bp{
        .tree = std::move(tree), .catalog = std::move(catalog), .earth = earth, .moon = moon};
}

} // namespace helios::tools

#endif // HELIOS_TOOLS_SUPPORT_UNIVERSES_HPP
