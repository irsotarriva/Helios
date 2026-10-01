#ifndef HELIOS_TEST_VESSEL_SUPPORT_HPP
#define HELIOS_TEST_VESSEL_SUPPORT_HPP

#include "helios/vessel/blueprint.hpp"
#include "helios/vessel/part_datasheet.hpp"
#include "helios/vessel/vessel_systems.hpp"

#include <filesystem>
#include <gtest/gtest.h>
#include <string_view>
#include <utility>

namespace helios::test {

// The stock parts of data/parts.
[[nodiscard]] inline vessel::PartCatalog load_stock_parts() {
    vessel::PartCatalog catalog;
    const auto loaded = catalog.load_directory(std::filesystem::path{HELIOS_DATA_DIR} / "parts");
    EXPECT_TRUE(loaded.has_value()) << (loaded ? "" : core::describe(loaded.error()));
    return catalog;
}

// A vessel of data/vessels/demo.toml, with its systems started at `start`.
[[nodiscard]] inline vessel::VesselSystems make_demo_vessel(const vessel::PartCatalog& catalog,
                                                            std::string_view name, const time::Epoch& start) {
    auto blueprints =
        vessel::load_blueprints(std::filesystem::path{HELIOS_DATA_DIR} / "vessels" / "demo.toml", catalog);
    EXPECT_TRUE(blueprints.has_value()) << (blueprints ? "" : core::describe(blueprints.error()));
    for (vessel::VesselBlueprint& blueprint : *blueprints) {
        if (blueprint.name == name) {
            return vessel::VesselSystems::make(std::move(blueprint.assembly), catalog.interfaces(),
                                               std::move(blueprint.stages), start)
                .value();
        }
    }
    ADD_FAILURE() << "no demo vessel named " << name;
    return vessel::VesselSystems::make({}, {}, {}, start).value();
}

[[nodiscard]] inline time::Epoch after(const time::Epoch& epoch, double seconds) {
    return epoch.advanced_by(seconds).value();
}

} // namespace helios::test

#endif // HELIOS_TEST_VESSEL_SUPPORT_HPP
