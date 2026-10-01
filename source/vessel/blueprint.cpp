#include "helios/vessel/blueprint.hpp"

#include "helios/core/parse.hpp"
#include "helios/core/toml.hpp"

#include <format>
#include <numbers>
#include <utility>

#include "toml_vector.hpp"

namespace helios::vessel {

namespace {

using core::ErrorCode;
using core::TomlValue;
using detail::vector_or;
using math::Vector3;

constexpr double k_degree_to_rad = std::numbers::pi / 180.0;

[[nodiscard]] core::VoidResult add_part(const TomlValue& table, const PartCatalog& catalog,
                                        Assembly& assembly) {
    if (core::VoidResult known =
            table.expect_keys({"name", "part", "parent", "position_m", "rotation_axis", "rotation_deg"});
        !known) {
        return known;
    }
    PartInstance instance;
    auto name = table.string_at("name");
    const auto datasheet =
        table.string_at("part").and_then([&](const std::string& id) { return catalog.part(id); });
    const auto position = vector_or(table, "position_m", Vector3{});
    const auto axis = vector_or(table, "rotation_axis", Vector3{0.0, 0.0, 1.0});
    const auto angle_deg = table.number_or("rotation_deg", 0.0);
    if (const auto error = core::first_error(name, datasheet, position, axis, angle_deg)) {
        return std::unexpected(*error);
    }
    instance.name = std::move(*name);
    instance.datasheet = *datasheet;
    instance.position_in_parent_m = *position;
    const auto orientation = rotation_about(*axis, *angle_deg * k_degree_to_rad);
    if (!orientation) {
        return std::unexpected(orientation.error());
    }
    instance.orientation_in_parent = *orientation;
    if (table.contains("parent")) {
        const auto parent = table.string_at("parent").and_then(
            [&](const std::string& parent_name) { return assembly.find(parent_name); });
        if (!parent) {
            return std::unexpected(parent.error());
        }
        instance.parent = *parent;
    }
    return assembly.add_part(std::move(instance)).transform([](std::size_t) {});
}

[[nodiscard]] core::Result<Stage> parse_stage(const TomlValue& table) {
    if (core::VoidResult known = table.expect_keys({"commands"}); !known) {
        return std::unexpected(known.error());
    }
    const auto rows = table.array_or_empty("commands");
    if (!rows) {
        return std::unexpected(rows.error());
    }
    Stage stage;
    for (const TomlValue& row : *rows) {
        if (core::VoidResult known = row.expect_keys({"signal", "value"}); !known) {
            return std::unexpected(known.error());
        }
        auto signal = row.string_at("signal");
        const auto value = row.number_at("value");
        if (const auto error = core::first_error(signal, value)) {
            return std::unexpected(*error);
        }
        stage.commands.push_back(StageCommand{.signal = std::move(*signal), .value = *value});
    }
    return stage;
}

} // namespace

core::Result<std::vector<VesselBlueprint>> parse_blueprints(std::string_view text,
                                                            const PartCatalog& catalog) {
    const auto document = core::parse_toml(text);
    if (!document) {
        return std::unexpected(document.error());
    }
    if (core::VoidResult known = document->expect_keys({"vessel"}); !known) {
        return std::unexpected(known.error());
    }
    const auto vessels = document->array_or_empty("vessel");
    if (!vessels) {
        return std::unexpected(vessels.error());
    }
    std::vector<VesselBlueprint> blueprints;
    for (const TomlValue& table : *vessels) {
        if (core::VoidResult known = table.expect_keys({"name", "part", "stage"}); !known) {
            return std::unexpected(known.error());
        }
        VesselBlueprint blueprint;
        auto name = table.string_at("name");
        const auto parts = table.array_or_empty("part");
        const auto stages = table.array_or_empty("stage");
        if (const auto error = core::first_error(name, parts, stages)) {
            return std::unexpected(*error);
        }
        blueprint.name = std::move(*name);
        if (parts->empty()) {
            return core::fail(ErrorCode::ParseFailure,
                              std::format("vessel '{}' has no parts", blueprint.name));
        }
        for (const TomlValue& part : *parts) {
            if (core::VoidResult added = add_part(part, catalog, blueprint.assembly); !added) {
                return core::fail(ErrorCode::ParseFailure,
                                  std::format("vessel '{}', part at line {}: {}", blueprint.name, part.line,
                                              added.error().context));
            }
        }
        for (const TomlValue& row : *stages) {
            auto stage = parse_stage(row);
            if (!stage) {
                return std::unexpected(stage.error());
            }
            blueprint.stages.push_back(std::move(*stage));
        }
        blueprints.push_back(std::move(blueprint));
    }
    return blueprints;
}

core::Result<std::vector<VesselBlueprint>> load_blueprints(const std::filesystem::path& path,
                                                           const PartCatalog& catalog) {
    return core::read_text_file(path)
        .and_then([&](const std::string& text) { return parse_blueprints(text, catalog); })
        .transform_error([&](core::Error error) {
            error.context = std::format("{}: {}", path.filename().string(), error.context);
            return error;
        });
}

} // namespace helios::vessel
