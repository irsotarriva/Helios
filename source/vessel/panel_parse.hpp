#ifndef HELIOS_SOURCE_VESSEL_PANEL_PARSE_HPP
#define HELIOS_SOURCE_VESSEL_PANEL_PARSE_HPP

#include "helios/core/error.hpp"
#include "helios/core/toml.hpp"
#include "helios/vessel/controls.hpp"

#include <string>
#include <vector>

// What every element of a cockpit's panels reads the same way: its look, its classes and its
// place (data/parts/README.md).
namespace helios::vessel::detail {

// The `look` table of an element; none: an empty patch.
[[nodiscard]] core::Result<LookPatch> look_or_empty(const core::TomlValue& table);
// `class = "red large"`: names separated by spaces.
[[nodiscard]] core::Result<std::vector<std::string>> classes_of(const core::TomlValue& table);
// `position_m`, `facing` and `up`, made unit and perpendicular.
[[nodiscard]] core::Result<PanelPlace> parse_place(const core::TomlValue& table);

} // namespace helios::vessel::detail

#endif // HELIOS_SOURCE_VESSEL_PANEL_PARSE_HPP
