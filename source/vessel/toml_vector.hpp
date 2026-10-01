#ifndef HELIOS_SOURCE_VESSEL_TOML_VECTOR_HPP
#define HELIOS_SOURCE_VESSEL_TOML_VECTOR_HPP

#include "helios/core/error.hpp"
#include "helios/core/toml.hpp"
#include "helios/math/vector3.hpp"

#include <format>
#include <string_view>
#include <vector>

namespace helios::vessel::detail {

// A member of three numbers as a vector; `fallback` when the key is absent.
[[nodiscard]] inline core::Result<math::Vector3> vector_or(const core::TomlValue& table, std::string_view key,
                                                           const math::Vector3& fallback) {
    if (!table.contains(key)) {
        return fallback;
    }
    return table.numbers_or_empty(key).and_then(
        [&](const std::vector<double>& numbers) -> core::Result<math::Vector3> {
            if (numbers.size() != 3) {
                return core::fail(
                    core::ErrorCode::ParseFailure,
                    std::format("'{}' in the table at line {} must have 3 numbers", key, table.line));
            }
            return math::Vector3{numbers[0], numbers[1], numbers[2]};
        });
}

} // namespace helios::vessel::detail

#endif // HELIOS_SOURCE_VESSEL_TOML_VECTOR_HPP
