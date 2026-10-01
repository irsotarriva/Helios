#include "helios/vessel/part_datasheet.hpp"

#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <string>

#include "support.hpp"

namespace {

using helios::core::ErrorCode;
using helios::vessel::Curve;
using helios::vessel::OutputPort;
using helios::vessel::PartCatalog;

constexpr const char* k_minimal_catalog = R"(
[[resource]]
id = "fuel"
mass_per_unit_kg = 1.0

[[interface]]
id = "engine"
inputs = ["throttle"]
outputs = ["thrust_n"]
)";

[[nodiscard]] helios::core::VoidResult load_with_part(const std::string& part_toml) {
    PartCatalog catalog;
    return catalog.load_toml(std::string{k_minimal_catalog} + part_toml);
}

TEST(Curve, InterpolatesAndHoldsBeyondItsEnds) {
    const Curve curve = Curve::make({{0.0, 10.0}, {1.0, 20.0}, {3.0, 0.0}}).value();
    EXPECT_DOUBLE_EQ(curve(-5.0), 10.0);
    EXPECT_DOUBLE_EQ(curve(0.5), 15.0);
    EXPECT_DOUBLE_EQ(curve(2.0), 10.0);
    EXPECT_DOUBLE_EQ(curve(99.0), 0.0);
    EXPECT_DOUBLE_EQ(Curve::constant(7.0)(123.0), 7.0);
}

TEST(Curve, RejectsEmptyUnorderedAndNonFinitePoints) {
    EXPECT_EQ(Curve::make({}).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(Curve::make({{1.0, 0.0}, {1.0, 2.0}}).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(Curve::make({{0.0, std::nan("")}}).error().code, ErrorCode::NotFinite);
}

TEST(PartCatalog, LoadsTheStockParts) {
    const PartCatalog catalog = helios::test::load_stock_parts();
    EXPECT_GE(catalog.parts().size(), 8U);
    const auto engine = catalog.part("core.engine_kerolox_vacuum");
    ASSERT_TRUE(engine.has_value());
    const helios::vessel::PartDatasheet& datasheet = **engine;
    EXPECT_EQ(datasheet.dry_mass_kg, 180.0);
    ASSERT_EQ(datasheet.processes.size(), 1U);
    const helios::vessel::Process& burn = datasheet.processes.front();
    EXPECT_EQ(burn.thrust_n, 30'000.0);
    EXPECT_EQ(burn.minimum_level, 0.4);
    ASSERT_EQ(burn.consumes.size(), 2U);
    // Masses per unit come from the resource definitions.
    EXPECT_EQ(burn.consumes[0].mass_per_unit_kg, 1.0);
    EXPECT_EQ(datasheet.inputs[*burn.level_input].name, "throttle");
    EXPECT_EQ(datasheet.inputs[*burn.enable_input].name, "ignition");
    // A solid cylinder about its axis: r² / 2 per unit of mass.
    EXPECT_DOUBLE_EQ(datasheet.gyration_m2.x, 0.125);
    const auto temperature = std::ranges::find(datasheet.outputs, "chamber_temperature_k", &OutputPort::name);
    ASSERT_NE(temperature, datasheet.outputs.end());
    EXPECT_EQ(temperature->quantity, OutputPort::Quantity::Curve);
    EXPECT_DOUBLE_EQ((*temperature->curve)(0.7), 3475.0);

    const auto ion = catalog.part("core.ion_thruster");
    ASSERT_TRUE(ion.has_value());
    EXPECT_EQ((*ion)->processes.front().consumes[1].mass_per_unit_kg, 0.0); // electric charge
    EXPECT_FALSE((*catalog.part("core.decoupler"))->crossfeed);
    EXPECT_EQ(catalog.part("core.warp_drive").error().code, ErrorCode::InvalidArgument);
}

TEST(PartCatalog, RejectsInvalidDatasheets) {
    for (const char* part : {
             // No mass.
             "[[part]]\nid = \"a\"\n",
             // A misspelt key.
             "[[part]]\nid = \"a\"\ndry_mass_kg = 1\ndry_mas_kg = 2\n",
             // An unknown resource.
             "[[part]]\nid = \"a\"\ndry_mass_kg = 1\nstores = [{ resource = \"air\", capacity = 1 }]\n",
             // More in the store than it holds.
             "[[part]]\nid = \"a\"\ndry_mass_kg = 1\n"
             "stores = [{ resource = \"fuel\", capacity = 1, initial = 2 }]\n",
             // A process driven by an input the part does not have.
             "[[part]]\nid = \"a\"\ndry_mass_kg = 1\n[[part.process]]\nname = \"p\"\nlevel = \"throttle\"\n",
             // An interface whose ports are missing.
             "[[part]]\nid = \"a\"\ndry_mass_kg = 1\ninterfaces = [\"engine\"]\n",
             // An unknown interface.
             "[[part]]\nid = \"a\"\ndry_mass_kg = 1\ninterfaces = [\"wing\"]\n",
             // An output of a process that does not exist.
             "[[part]]\nid = \"a\"\ndry_mass_kg = 1\n"
             "[[part.output]]\nname = \"t\"\nprocess = \"p\"\nquantity = \"thrust\"\n",
             // A flow without a positive rate.
             "[[part]]\nid = \"a\"\ndry_mass_kg = 1\n[[part.process]]\nname = \"p\"\n"
             "consumes = [{ resource = \"fuel\", rate_per_s = 0 }]\n",
             // Two parts with one id.
             "[[part]]\nid = \"a\"\ndry_mass_kg = 1\n[[part]]\nid = \"a\"\ndry_mass_kg = 1\n",
         }) {
        const auto loaded = load_with_part(part);
        EXPECT_FALSE(loaded.has_value()) << part;
    }
}

TEST(PartCatalog, AFailedLoadAddsNothing) {
    PartCatalog catalog;
    ASSERT_TRUE(catalog.load_toml(k_minimal_catalog).has_value());
    const auto loaded = catalog.load_toml("[[resource]]\nid = \"air\"\nmass_per_unit_kg = 1\n"
                                          "[[part]]\nid = \"broken\"\n");
    ASSERT_FALSE(loaded.has_value());
    EXPECT_EQ(catalog.resources().size(), 1U);
    EXPECT_TRUE(catalog.parts().empty());
    EXPECT_EQ(catalog.load_directory("no/such/directory").error().code, ErrorCode::FileNotFound);
}

} // namespace
