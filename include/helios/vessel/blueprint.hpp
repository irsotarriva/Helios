#ifndef HELIOS_VESSEL_BLUEPRINT_HPP
#define HELIOS_VESSEL_BLUEPRINT_HPP

#include "helios/core/error.hpp"
#include "helios/vessel/assembly.hpp"
#include "helios/vessel/part_datasheet.hpp"
#include "helios/vessel/vessel_systems.hpp"

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace helios::vessel {

// A vessel as designed: which parts, where, and what its stages do. Instantiated into
// VesselSystems when it is launched.
struct VesselBlueprint {
    std::string name;
    Assembly assembly;
    std::vector<Stage> stages;
};

// The [[vessel]] tables of a TOML document (format: data/vessels/README.md), with parts taken
// from `catalog`.
[[nodiscard]] core::Result<std::vector<VesselBlueprint>> parse_blueprints(std::string_view text,
                                                                          const PartCatalog& catalog);
[[nodiscard]] core::Result<std::vector<VesselBlueprint>> load_blueprints(const std::filesystem::path& path,
                                                                         const PartCatalog& catalog);

} // namespace helios::vessel

#endif // HELIOS_VESSEL_BLUEPRINT_HPP
