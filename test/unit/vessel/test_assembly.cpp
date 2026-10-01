#include "helios/vessel/assembly.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace {

using helios::core::ErrorCode;
using helios::math::Matrix3;
using helios::math::Vector3;
using helios::vessel::Assembly;
using helios::vessel::PartDatasheet;
using helios::vessel::PartInstance;

[[nodiscard]] std::shared_ptr<const PartDatasheet> point_mass(double mass_kg) {
    PartDatasheet datasheet;
    datasheet.id = "point";
    datasheet.dry_mass_kg = mass_kg;
    return std::make_shared<const PartDatasheet>(std::move(datasheet));
}

[[nodiscard]] PartInstance instance(std::string name, std::shared_ptr<const PartDatasheet> datasheet,
                                    std::optional<std::size_t> parent, const Vector3& position_m) {
    return PartInstance{.name = std::move(name),
                        .datasheet = std::move(datasheet),
                        .parent = parent,
                        .position_in_parent_m = position_m,
                        .orientation_in_parent = {}};
}

TEST(Assembly, CompositeMassPropertiesOfTwoPointMasses) {
    Assembly assembly;
    ASSERT_TRUE(assembly.add_part(instance("a", point_mass(1.0), {}, {})).has_value());
    ASSERT_TRUE(assembly.add_part(instance("b", point_mass(3.0), 0, {4.0, 0.0, 0.0})).has_value());
    const std::vector<double> masses_kg{1.0, 3.0};
    const auto properties = assembly.mass_properties(masses_kg);
    ASSERT_TRUE(properties.has_value());
    EXPECT_DOUBLE_EQ(properties->mass_kg, 4.0);
    EXPECT_DOUBLE_EQ(properties->centre_of_mass_m.x, 3.0);
    // A dumbbell: the reduced mass 3/4 kg at 4 m, about the two axes across it.
    EXPECT_DOUBLE_EQ(properties->inertia_kg_m2(0, 0), 0.0);
    EXPECT_DOUBLE_EQ(properties->inertia_kg_m2(1, 1), 12.0);
    EXPECT_DOUBLE_EQ(properties->inertia_kg_m2(2, 2), 12.0);
    EXPECT_DOUBLE_EQ(properties->inertia_kg_m2(0, 1), 0.0);
}

TEST(Assembly, PartInertiaIsTurnedWithThePart) {
    PartDatasheet rod;
    rod.id = "rod";
    rod.dry_mass_kg = 2.0;
    rod.gyration_m2 = Vector3{0.0, 3.0, 3.0}; // a thin rod along its own x
    const auto datasheet = std::make_shared<const PartDatasheet>(std::move(rod));
    Assembly assembly;
    ASSERT_TRUE(assembly.add_part(instance("root", point_mass(1e-9), {}, {})).has_value());
    PartInstance turned = instance("rod", datasheet, 0, {});
    turned.orientation_in_parent =
        helios::vessel::rotation_about({0.0, 0.0, 1.0}, std::numbers::pi / 2.0).value();
    ASSERT_TRUE(assembly.add_part(std::move(turned)).has_value());
    // Turned a quarter about z, the rod lies along the vessel's y.
    const Vector3 along = assembly.poses()[1].orientation * Vector3{1.0, 0.0, 0.0};
    EXPECT_NEAR(along.x, 0.0, 1e-15);
    EXPECT_NEAR(along.y, 1.0, 1e-15);
    const std::vector<double> masses_kg{1e-9, 2.0};
    const auto properties = assembly.mass_properties(masses_kg).value();
    EXPECT_NEAR(properties.inertia_kg_m2(0, 0), 6.0, 1e-12);
    EXPECT_NEAR(properties.inertia_kg_m2(1, 1), 0.0, 1e-12);
    EXPECT_NEAR(properties.inertia_kg_m2(2, 2), 6.0, 1e-12);
}

TEST(Assembly, ChildrenArePlacedThroughTheirParents) {
    Assembly assembly;
    ASSERT_TRUE(assembly.add_part(instance("root", point_mass(1.0), {}, {})).has_value());
    PartInstance arm = instance("arm", point_mass(1.0), 0, {1.0, 0.0, 0.0});
    arm.orientation_in_parent =
        helios::vessel::rotation_about({0.0, 0.0, 1.0}, std::numbers::pi / 2.0).value();
    ASSERT_TRUE(assembly.add_part(std::move(arm)).has_value());
    ASSERT_TRUE(assembly.add_part(instance("tip", point_mass(1.0), 1, {2.0, 0.0, 0.0})).has_value());
    // The tip is 2 m along the arm's x, which is the vessel's y.
    EXPECT_NEAR(assembly.poses()[2].position_m.x, 1.0, 1e-15);
    EXPECT_NEAR(assembly.poses()[2].position_m.y, 2.0, 1e-15);
}

TEST(Assembly, SplitsAtAPartKeepingBothSidesConsistent) {
    Assembly assembly;
    ASSERT_TRUE(assembly.add_part(instance("probe", point_mass(1.0), {}, {})).has_value());
    ASSERT_TRUE(assembly.add_part(instance("tank", point_mass(1.0), 0, {-1.0, 0.0, 0.0})).has_value());
    ASSERT_TRUE(assembly.add_part(instance("decoupler", point_mass(1.0), 1, {-1.0, 0.0, 0.0})).has_value());
    ASSERT_TRUE(assembly.add_part(instance("antenna", point_mass(1.0), 0, {0.0, 1.0, 0.0})).has_value());
    ASSERT_TRUE(assembly.add_part(instance("booster", point_mass(1.0), 2, {-3.0, 0.0, 0.0})).has_value());
    EXPECT_EQ(assembly.subtree(2), (std::vector<std::size_t>{2, 4}));

    const auto split = assembly.split_at(2);
    ASSERT_TRUE(split.has_value());
    EXPECT_EQ(split->kept_parts, (std::vector<std::size_t>{0, 1, 3}));
    EXPECT_EQ(split->separated_parts, (std::vector<std::size_t>{2, 4}));
    ASSERT_EQ(split->kept.parts().size(), 3U);
    EXPECT_EQ(split->kept.parts()[2].name, "antenna");
    EXPECT_EQ(split->kept.parts()[2].parent, 0U);
    // The detached part is the root of the new vessel, at its origin.
    ASSERT_EQ(split->separated.parts().size(), 2U);
    EXPECT_EQ(split->separated.parts()[0].name, "decoupler");
    EXPECT_FALSE(split->separated.parts()[0].parent.has_value());
    EXPECT_EQ(split->separated.parts()[1].parent, 0U);
    EXPECT_DOUBLE_EQ(split->separated.poses()[1].position_m.x, -3.0);
}

TEST(Assembly, RejectsInvalidParts) {
    Assembly assembly;
    EXPECT_EQ(assembly.add_part(instance("a", nullptr, {}, {})).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(assembly.add_part(instance("a", point_mass(1.0), 0, {})).error().code,
              ErrorCode::InvalidArgument); // the root has no parent
    ASSERT_TRUE(assembly.add_part(instance("a", point_mass(1.0), {}, {})).has_value());
    EXPECT_EQ(assembly.add_part(instance("a", point_mass(1.0), 0, {})).error().code,
              ErrorCode::InvalidArgument); // duplicate name
    EXPECT_EQ(assembly.add_part(instance("b/c", point_mass(1.0), 0, {})).error().code,
              ErrorCode::InvalidArgument); // '/' separates part and port in signal names
    EXPECT_EQ(assembly.add_part(instance("b", point_mass(1.0), {}, {})).error().code,
              ErrorCode::InvalidArgument); // a second root
    EXPECT_EQ(assembly.add_part(instance("b", point_mass(1.0), 7, {})).error().code, ErrorCode::OutOfRange);
    PartInstance skewed = instance("b", point_mass(1.0), 0, {});
    skewed.orientation_in_parent = Matrix3{{2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
    EXPECT_EQ(assembly.add_part(std::move(skewed)).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(assembly.split_at(0).error().code, ErrorCode::OutOfRange);
    EXPECT_EQ(assembly.mass_properties({}).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(helios::vessel::rotation_about({}, 1.0).error().code, ErrorCode::InvalidArgument);
}

} // namespace
