#include "helios/bodies/body_catalog.hpp"

#include <gtest/gtest.h>
#include <string>

#include "../support/universes.hpp"

namespace {

using helios::bodies::Body;
using helios::bodies::BodyCatalog;
using helios::bodies::parse_bodies_csv;
using helios::core::ErrorCode;
using helios::frames::FixedOffset;
using helios::frames::FrameTree;

constexpr std::string_view k_header =
    "name,frame,gravitational_parameter_m3_s2,mean_radius_m,domain_parent,domain_radius_m,pole_ra_deg,"
    "pole_ra_rate_deg_per_century,pole_dec_deg,pole_dec_rate_deg_per_century,prime_meridian_deg,"
    "rotation_rate_deg_per_day\n";

[[nodiscard]] FrameTree two_frames() {
    FrameTree tree("Star");
    static_cast<void>(tree.add_frame("Planet", FrameTree::root(), FixedOffset{{1e11, 0.0, 0.0}}).value());
    return tree;
}

TEST(BodyCatalog, ParsesBodiesDomainsAndRotation) {
    const FrameTree tree = two_frames();
    const std::string csv = std::string{k_header} + "Star,Star,1e20,7e8,,,0,0,90,0,0,14\n"
                            + "Planet,Planet,4e14,6.4e6,Star,9e8,0,0,90,0,0,360\n";
    const auto catalog = parse_bodies_csv(csv, tree, {});
    ASSERT_TRUE(catalog.has_value()) << helios::core::describe(catalog.error());
    const auto planet = catalog->find("Planet").value();
    const auto planet_reference = catalog->body(planet).value();
    const Body& planet_body = planet_reference.get();
    EXPECT_EQ(planet_body.domain_parent, catalog->find("Star").value());
    EXPECT_DOUBLE_EQ(planet_body.domain_radius_m, 9e8);
    ASSERT_TRUE(planet_body.rotation.has_value());
    EXPECT_NEAR(planet_body.rotation->sidereal_period_s(), 86'400.0, 1e-9);
    EXPECT_EQ(catalog->domain_children(catalog->find("Star").value()).size(), 1U);
    EXPECT_EQ(catalog->body_on_frame(planet_body.frame), planet);
}

TEST(BodyCatalog, RejectsBadRows) {
    const FrameTree tree = two_frames();
    const std::string star = "Star,Star,1e20,7e8,,,,,,,,\n";
    EXPECT_EQ(
        parse_bodies_csv(std::string{k_header} + "Star,Nowhere,1e20,7e8,,,,,,,,\n", tree, {}).error().code,
        ErrorCode::ParseFailure);
    EXPECT_EQ(parse_bodies_csv(std::string{k_header} + star + star, tree, {}).error().code,
              ErrorCode::ParseFailure);
    EXPECT_EQ(
        parse_bodies_csv(std::string{k_header} + star + "Planet,Planet,4e14,6.4e6,Star,,,,,,,\n", tree, {})
            .error()
            .code,
        ErrorCode::ParseFailure);
    EXPECT_EQ(parse_bodies_csv(std::string{k_header} + "Star,Star,abc,7e8,,,,,,,,\n", tree, {}).error().code,
              ErrorCode::ParseFailure);
    EXPECT_EQ(parse_bodies_csv(std::string{k_header} + "Star,Star,1e20\n", tree, {}).error().code,
              ErrorCode::ParseFailure);
}

TEST(BodyCatalog, StockSolarSystemLoads) {
    const auto solar_system = helios::test::load_solar_system();
    const BodyCatalog& catalog = *solar_system.catalog;
    EXPECT_EQ(catalog.bodies().size(), 11U);
    const auto earth = catalog.find("Earth").value();
    const auto moon = catalog.find("Moon").value();
    EXPECT_EQ(catalog.body(moon).value().get().domain_parent, earth);
    EXPECT_EQ(catalog.body(earth).value().get().domain_parent, catalog.find("Sun").value());
    // Earth's sphere of influence ≈ 0.925 million km.
    EXPECT_NEAR(catalog.body(earth).value().get().domain_radius_m, 9.25e8, 0.01e8);
}

} // namespace
