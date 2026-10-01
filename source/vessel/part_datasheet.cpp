#include "helios/vessel/part_datasheet.hpp"

#include "helios/core/parse.hpp"
#include "helios/core/toml.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <functional>
#include <iterator>
#include <utility>

namespace helios::vessel {

namespace {

using core::ErrorCode;
using core::first_error;
using core::TomlValue;
using math::Vector3;

[[nodiscard]] bool is_finite(const Vector3& vector) noexcept {
    return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

template <typename Range>
[[nodiscard]] bool has_duplicate_names(const Range& range) {
    for (auto first = range.begin(); first != range.end(); ++first) {
        for (auto second = std::next(first); second != range.end(); ++second) {
            if (first->name == second->name) {
                return true;
            }
        }
    }
    return false;
}

[[nodiscard]] std::unexpected<core::Error> invalid(std::string_view part_id, std::string_view message) {
    return core::fail(ErrorCode::InvalidArgument, std::format("part '{}': {}", part_id, message));
}

[[nodiscard]] core::Result<Vector3> vector_or(const TomlValue& table, std::string_view key,
                                              const Vector3& fallback) {
    if (!table.contains(key)) {
        return fallback;
    }
    return table.numbers_or_empty(key).and_then(
        [&](const std::vector<double>& numbers) -> core::Result<Vector3> {
            if (numbers.size() != 3) {
                return core::fail(
                    ErrorCode::ParseFailure,
                    std::format("'{}' in the table at line {} must have 3 numbers", key, table.line));
            }
            return Vector3{numbers[0], numbers[1], numbers[2]};
        });
}

[[nodiscard]] core::Result<Curve> parse_curve(const TomlValue& table, std::string_view key) {
    const auto rows = table.array_or_empty(key);
    if (!rows) {
        return std::unexpected(rows.error());
    }
    std::vector<Curve::Point> points;
    for (const TomlValue& row : *rows) {
        if (row.kind != TomlValue::Kind::Array || row.items.size() != 2
            || row.items[0].kind != TomlValue::Kind::Number || row.items[1].kind != TomlValue::Kind::Number) {
            return core::fail(ErrorCode::ParseFailure,
                              std::format("'{}' at line {} must be a list of [x, y] pairs", key, row.line));
        }
        points.push_back(Curve::Point{.x = row.items[0].number, .y = row.items[1].number});
    }
    return Curve::make(std::move(points));
}

template <typename Range>
[[nodiscard]] core::Result<std::size_t> index_of_name(const Range& range, std::string_view name,
                                                      std::string_view what, const TomlValue& table) {
    const auto match = std::ranges::find_if(range, [&](const auto& item) { return item.name == name; });
    if (match == range.end()) {
        return core::fail(ErrorCode::ParseFailure,
                          std::format("line {}: there is no {} named '{}'", table.line, what, name));
    }
    return static_cast<std::size_t>(match - range.begin());
}

[[nodiscard]] core::Result<std::vector<ResourceFlow>> parse_flows(const TomlValue& table,
                                                                  std::string_view key) {
    const auto rows = table.array_or_empty(key);
    if (!rows) {
        return std::unexpected(rows.error());
    }
    std::vector<ResourceFlow> flows;
    for (const TomlValue& row : *rows) {
        auto resource = row.string_at("resource");
        const auto rate = row.number_at("rate_per_s");
        if (const auto error = first_error(row.expect_keys({"resource", "rate_per_s"}), resource, rate)) {
            return std::unexpected(*error);
        }
        flows.push_back(
            ResourceFlow{.resource = std::move(*resource), .rate_per_s = *rate, .mass_per_unit_kg = 0.0});
    }
    return flows;
}

[[nodiscard]] core::Result<InputPort> parse_input(const TomlValue& table) {
    auto name = table.string_at("name");
    auto unit = table.string_or("unit", "");
    const auto minimum = table.number_or("minimum", 0.0);
    const auto maximum = table.number_or("maximum", 1.0);
    const auto default_value = table.number_or("default", minimum.value_or(0.0));
    const auto toggle = table.boolean_or("toggle", false);
    if (const auto error =
            first_error(table.expect_keys({"name", "unit", "minimum", "maximum", "default", "toggle"}), name,
                        unit, minimum, maximum, default_value, toggle)) {
        return std::unexpected(*error);
    }
    return InputPort{.name = std::move(*name),
                     .unit = std::move(*unit),
                     .minimum = *minimum,
                     .maximum = *maximum,
                     .default_value = *default_value,
                     .toggle = *toggle};
}

[[nodiscard]] core::Result<Store> parse_store(const TomlValue& table) {
    auto resource = table.string_at("resource");
    const auto capacity = table.number_at("capacity");
    const auto initial = table.number_or("initial", capacity.value_or(0.0));
    if (const auto error = first_error(table.expect_keys({"resource", "capacity", "initial"}), resource,
                                       capacity, initial)) {
        return std::unexpected(*error);
    }
    return Store{.resource = std::move(*resource),
                 .capacity = *capacity,
                 .initial_amount = *initial,
                 .mass_per_unit_kg = 0.0};
}

[[nodiscard]] core::Result<Process> parse_process(const TomlValue& table, const PartDatasheet& part) {
    const auto input_named_by = [&](std::string_view key) -> core::Result<std::optional<std::size_t>> {
        if (!table.contains(key)) {
            return std::nullopt;
        }
        return table.string_at(key)
            .and_then([&](const std::string& input_name) {
                return index_of_name(part.inputs, input_name, "input", table);
            })
            .transform([](std::size_t index) { return std::optional{index}; });
    };
    auto name = table.string_at("name");
    const auto level_input = input_named_by("level");
    const auto enable_input = input_named_by("enable");
    const auto minimum_level = table.number_or("minimum_level", 0.0);
    const auto thrust = table.number_or("thrust_n", 0.0);
    auto consumes = parse_flows(table, "consumes");
    auto produces = parse_flows(table, "produces");
    const auto direction = vector_or(table, "thrust_direction", Vector3{1.0, 0.0, 0.0});
    const auto position = vector_or(table, "thrust_position_m", Vector3{});
    if (const auto error = first_error(
            table.expect_keys({"name", "level", "enable", "minimum_level", "consumes", "produces", "thrust_n",
                               "thrust_curve", "thrust_direction", "thrust_position_m"}),
            name, level_input, enable_input, minimum_level, thrust, consumes, produces, direction,
            position)) {
        return std::unexpected(*error);
    }
    Process process{.name = std::move(*name),
                    .level_input = *level_input,
                    .enable_input = *enable_input,
                    .minimum_level = *minimum_level,
                    .consumes = std::move(*consumes),
                    .produces = std::move(*produces),
                    .thrust_n = *thrust,
                    .thrust_curve = {},
                    .thrust_direction = *direction,
                    .thrust_position_m = *position};
    if (table.contains("thrust_curve")) {
        auto curve = parse_curve(table, "thrust_curve");
        if (!curve) {
            return std::unexpected(curve.error());
        }
        process.thrust_curve = std::move(*curve);
    }
    return process;
}

[[nodiscard]] core::Result<OutputPort> parse_output(const TomlValue& table, const PartDatasheet& part) {
    using Quantity = OutputPort::Quantity;
    constexpr std::array<std::pair<std::string_view, Quantity>, 6> k_quantities{{
        {"level", Quantity::Level},
        {"thrust", Quantity::Thrust},
        {"mass_flow", Quantity::MassFlow},
        {"curve", Quantity::Curve},
        {"amount", Quantity::Amount},
        {"fraction", Quantity::Fraction},
    }};
    auto name = table.string_at("name");
    auto unit = table.string_or("unit", "");
    const auto quantity_name = table.string_or("quantity", table.contains("curve") ? "curve" : "");
    if (const auto error =
            first_error(table.expect_keys({"name", "unit", "process", "store", "quantity", "curve"}), name,
                        unit, quantity_name)) {
        return std::unexpected(*error);
    }
    std::optional<Quantity> quantity;
    for (const auto& [label, value] : k_quantities) {
        if (label == *quantity_name) {
            quantity = value;
        }
    }
    if (!quantity.has_value()) {
        return core::fail(ErrorCode::ParseFailure,
                          std::format("line {}: output '{}' needs a quantity (level, thrust, mass_flow, "
                                      "curve, amount or fraction)",
                                      table.line, *name));
    }
    OutputPort output{
        .name = std::move(*name), .unit = std::move(*unit), .quantity = *quantity, .index = 0, .curve = {}};
    const bool of_store = output.quantity == Quantity::Amount || output.quantity == Quantity::Fraction;
    const auto owner = table.string_at(of_store ? "store" : "process");
    if (!owner) {
        return std::unexpected(owner.error());
    }
    if (of_store) {
        const auto store = std::ranges::find(part.stores, *owner, &Store::resource);
        if (store == part.stores.end()) {
            return core::fail(ErrorCode::ParseFailure,
                              std::format("line {}: the part has no store of '{}'", table.line, *owner));
        }
        output.index = static_cast<std::size_t>(store - part.stores.begin());
    } else {
        const auto process = index_of_name(part.processes, *owner, "process", table);
        if (!process) {
            return std::unexpected(process.error());
        }
        output.index = *process;
    }
    if (output.quantity == Quantity::Curve) {
        auto curve = parse_curve(table, "curve");
        if (!curve) {
            return std::unexpected(curve.error());
        }
        output.curve = std::move(*curve);
    }
    return output;
}

// Moments of inertia per unit of mass of a solid shape whose axis is the part's x axis.
[[nodiscard]] core::Result<Vector3> parse_shape(const TomlValue& table) {
    const auto kind = table.string_at("kind");
    const auto radius_m = table.number_at("radius_m");
    const auto length_m = table.number_at("length_m");
    if (const auto error =
            first_error(table.expect_keys({"kind", "radius_m", "length_m"}), kind, radius_m, length_m)) {
        return std::unexpected(*error);
    }
    if (*kind != "cylinder") {
        return core::fail(ErrorCode::ParseFailure,
                          std::format("line {}: unknown shape '{}' (known: cylinder)", table.line, *kind));
    }
    const double radius_squared_m2 = *radius_m * *radius_m;
    const double transverse_m2 = radius_squared_m2 / 4.0 + *length_m * *length_m / 12.0;
    return Vector3{radius_squared_m2 / 2.0, transverse_m2, transverse_m2};
}

[[nodiscard]] core::Result<Separator> parse_separator(const TomlValue& table, const PartDatasheet& part) {
    const auto input = table.string_at("input").and_then([&](const std::string& input_name) {
        return index_of_name(part.inputs, input_name, "input", table);
    });
    const auto impulse = table.number_or("impulse_n_s", 0.0);
    if (const auto error = first_error(table.expect_keys({"input", "impulse_n_s"}), input, impulse)) {
        return std::unexpected(*error);
    }
    return Separator{.input = *input, .impulse_n_s = *impulse};
}

[[nodiscard]] core::Result<PartDatasheet> parse_part(const TomlValue& table) {
    auto id = table.string_at("id");
    const auto dry_mass = table.number_at("dry_mass_kg");
    auto name = table.string_or("name", id.value_or(""));
    const auto centre = vector_or(table, "centre_of_mass_m", Vector3{});
    const auto inertia = vector_or(table, "inertia_kg_m2", Vector3{});
    const auto crossfeed = table.boolean_or("crossfeed", true);
    auto interfaces = table.strings_or_empty("interfaces");
    const auto inputs = table.array_or_empty("inputs");
    const auto stores = table.array_or_empty("stores");
    const auto processes = table.array_or_empty("process");
    const auto outputs = table.array_or_empty("output");
    if (const auto error = first_error(
            table.expect_keys({"id", "name", "dry_mass_kg", "centre_of_mass_m", "inertia_kg_m2", "shape",
                               "crossfeed", "interfaces", "inputs", "stores", "separator", "process",
                               "output"}),
            id, dry_mass, name, centre, inertia, crossfeed, interfaces, inputs, stores, processes, outputs)) {
        return std::unexpected(*error);
    }
    PartDatasheet part;
    part.id = std::move(*id);
    part.name = std::move(*name);
    part.dry_mass_kg = *dry_mass;
    part.centre_of_mass_m = *centre;
    part.crossfeed = *crossfeed;
    part.interfaces = std::move(*interfaces);

    if (table.contains("shape") && table.contains("inertia_kg_m2")) {
        return core::fail(ErrorCode::ParseFailure,
                          std::format("part '{}': give either a shape or inertia_kg_m2", part.id));
    }
    if (table.contains("shape")) {
        const auto gyration = table.at("shape").and_then(parse_shape);
        if (!gyration) {
            return std::unexpected(gyration.error());
        }
        part.gyration_m2 = *gyration;
    } else {
        part.gyration_m2 = *inertia / part.dry_mass_kg;
    }

    for (const TomlValue& row : *inputs) {
        auto input = parse_input(row);
        if (!input) {
            return std::unexpected(input.error());
        }
        part.inputs.push_back(std::move(*input));
    }
    for (const TomlValue& row : *stores) {
        auto store = parse_store(row);
        if (!store) {
            return std::unexpected(store.error());
        }
        part.stores.push_back(std::move(*store));
    }
    for (const TomlValue& row : *processes) {
        auto process = parse_process(row, part);
        if (!process) {
            return std::unexpected(process.error());
        }
        part.processes.push_back(std::move(*process));
    }
    for (const TomlValue& row : *outputs) {
        auto output = parse_output(row, part);
        if (!output) {
            return std::unexpected(output.error());
        }
        part.outputs.push_back(std::move(*output));
    }
    if (table.contains("separator")) {
        const auto separator =
            table.at("separator").and_then([&](const TomlValue& row) { return parse_separator(row, part); });
        if (!separator) {
            return std::unexpected(separator.error());
        }
        part.separator = *separator;
    }
    return part;
}

} // namespace

core::VoidResult PartCatalog::add_resource(ResourceDefinition resource) {
    if (resource.id.empty()
        || std::ranges::find(resources_, resource.id, &ResourceDefinition::id) != resources_.end()) {
        return core::fail(ErrorCode::InvalidArgument,
                          std::format("resource id '{}' is empty or duplicated", resource.id));
    }
    if (!std::isfinite(resource.mass_per_unit_kg) || resource.mass_per_unit_kg < 0.0) {
        return core::fail(ErrorCode::OutOfRange,
                          std::format("resource '{}' needs a mass per unit of 0 or more", resource.id));
    }
    resources_.push_back(std::move(resource));
    return {};
}

core::VoidResult PartCatalog::add_interface(InterfaceDefinition definition) {
    if (definition.id.empty()
        || std::ranges::find(interfaces_, definition.id, &InterfaceDefinition::id) != interfaces_.end()) {
        return core::fail(ErrorCode::InvalidArgument,
                          std::format("interface id '{}' is empty or duplicated", definition.id));
    }
    interfaces_.push_back(std::move(definition));
    return {};
}

core::VoidResult PartCatalog::add_part(PartDatasheet part) {
    if (part.id.empty() || this->part(part.id).has_value()) {
        return core::fail(ErrorCode::InvalidArgument,
                          std::format("part id '{}' is empty or duplicated", part.id));
    }
    if (!std::isfinite(part.dry_mass_kg) || part.dry_mass_kg <= 0.0) {
        return invalid(part.id, "the dry mass must be positive");
    }
    if (!is_finite(part.centre_of_mass_m) || !is_finite(part.gyration_m2) || part.gyration_m2.x < 0.0
        || part.gyration_m2.y < 0.0 || part.gyration_m2.z < 0.0) {
        return invalid(part.id, "the centre of mass or the inertia is invalid");
    }
    if (has_duplicate_names(part.inputs) || has_duplicate_names(part.outputs)
        || has_duplicate_names(part.processes)) {
        return invalid(part.id, "two inputs, outputs or processes share a name");
    }
    for (const InputPort& input : part.inputs) {
        if (input.name.empty() || !std::isfinite(input.minimum) || !std::isfinite(input.maximum)
            || !(input.minimum < input.maximum) || !(input.default_value >= input.minimum)
            || !(input.default_value <= input.maximum)) {
            return invalid(part.id,
                           std::format("input '{}' needs minimum <= default <= maximum", input.name));
        }
        if (std::ranges::find(part.outputs, input.name, &OutputPort::name) != part.outputs.end()) {
            return invalid(part.id, std::format("'{}' is both an input and an output", input.name));
        }
    }
    const auto mass_per_unit_of = [&](std::string_view resource) -> core::Result<double> {
        const auto match = std::ranges::find(resources_, resource, &ResourceDefinition::id);
        if (match == resources_.end()) {
            return invalid(part.id, std::format("unknown resource '{}'", resource));
        }
        return match->mass_per_unit_kg;
    };
    for (std::size_t index = 0; index < part.stores.size(); ++index) {
        Store& store = part.stores[index];
        const auto mass_per_unit_kg = mass_per_unit_of(store.resource);
        if (!mass_per_unit_kg) {
            return std::unexpected(mass_per_unit_kg.error());
        }
        store.mass_per_unit_kg = *mass_per_unit_kg;
        if (!std::isfinite(store.capacity) || store.capacity <= 0.0 || !(store.initial_amount >= 0.0)
            || !(store.initial_amount <= store.capacity)) {
            return invalid(part.id,
                           std::format("the store of '{}' needs 0 <= initial <= capacity", store.resource));
        }
        const auto earlier = std::span{part.stores}.first(index);
        if (std::ranges::find(earlier, store.resource, &Store::resource) != earlier.end()) {
            return invalid(part.id, std::format("two stores hold '{}'", store.resource));
        }
    }
    for (Process& process : part.processes) {
        const bool inputs_exist =
            (!process.level_input.has_value() || *process.level_input < part.inputs.size())
            && (!process.enable_input.has_value() || *process.enable_input < part.inputs.size());
        if (process.name.empty() || !inputs_exist) {
            return invalid(part.id, "a process needs a name and existing inputs");
        }
        if (!(process.minimum_level >= 0.0) || !(process.minimum_level <= 1.0)
            || !std::isfinite(process.thrust_n) || process.thrust_n < 0.0
            || !is_finite(process.thrust_position_m)) {
            return invalid(part.id, std::format("process '{}' has an invalid level or thrust", process.name));
        }
        const double direction_norm = math::norm(process.thrust_direction);
        if (!std::isfinite(direction_norm) || direction_norm <= 0.0) {
            return invalid(part.id, std::format("process '{}' needs a thrust direction", process.name));
        }
        process.thrust_direction = process.thrust_direction / direction_norm;
        for (const std::reference_wrapper<std::vector<ResourceFlow>> flows :
             {std::ref(process.consumes), std::ref(process.produces)}) {
            for (ResourceFlow& flow : flows.get()) {
                const auto mass_per_unit_kg = mass_per_unit_of(flow.resource);
                if (!mass_per_unit_kg) {
                    return std::unexpected(mass_per_unit_kg.error());
                }
                flow.mass_per_unit_kg = *mass_per_unit_kg;
                if (!std::isfinite(flow.rate_per_s) || flow.rate_per_s <= 0.0) {
                    return invalid(part.id,
                                   std::format("the flow of '{}' needs a positive rate", flow.resource));
                }
            }
        }
    }
    for (const OutputPort& output : part.outputs) {
        const bool of_store = output.quantity == OutputPort::Quantity::Amount
                              || output.quantity == OutputPort::Quantity::Fraction;
        const std::size_t count = of_store ? part.stores.size() : part.processes.size();
        if (output.name.empty() || output.index >= count
            || (output.quantity == OutputPort::Quantity::Curve && !output.curve.has_value())) {
            return invalid(part.id, std::format("output '{}' refers to nothing", output.name));
        }
    }
    if (part.separator.has_value()
        && (part.separator->input >= part.inputs.size() || !std::isfinite(part.separator->impulse_n_s)
            || part.separator->impulse_n_s < 0.0)) {
        return invalid(part.id, "the separator needs an existing input and an impulse of 0 or more");
    }
    for (const std::string& interface_id : part.interfaces) {
        const auto definition = std::ranges::find(interfaces_, interface_id, &InterfaceDefinition::id);
        if (definition == interfaces_.end()) {
            return invalid(part.id, std::format("unknown interface '{}'", interface_id));
        }
        for (const std::string& input : definition->inputs) {
            if (std::ranges::find(part.inputs, input, &InputPort::name) == part.inputs.end()) {
                return invalid(part.id,
                               std::format("interface '{}' requires the input '{}'", interface_id, input));
            }
        }
        for (const std::string& output : definition->outputs) {
            if (std::ranges::find(part.outputs, output, &OutputPort::name) == part.outputs.end()) {
                return invalid(part.id,
                               std::format("interface '{}' requires the output '{}'", interface_id, output));
            }
        }
    }
    parts_.push_back(std::make_shared<const PartDatasheet>(std::move(part)));
    return {};
}

core::Result<std::shared_ptr<const PartDatasheet>> PartCatalog::part(std::string_view id) const {
    const auto match = std::ranges::find_if(
        parts_, [&](const std::shared_ptr<const PartDatasheet>& part) { return part->id == id; });
    if (match == parts_.end()) {
        return core::fail(ErrorCode::InvalidArgument, std::format("no part with the id '{}'", id));
    }
    return *match;
}

core::VoidResult PartCatalog::load_toml(std::string_view text) {
    const auto document = core::parse_toml(text);
    if (!document) {
        return std::unexpected(document.error());
    }
    const auto resources = document->array_or_empty("resource");
    const auto interfaces = document->array_or_empty("interface");
    const auto parts = document->array_or_empty("part");
    if (const auto error = first_error(document->expect_keys({"resource", "interface", "part"}), resources,
                                       interfaces, parts)) {
        return std::unexpected(*error);
    }
    PartCatalog updated = *this;
    for (const TomlValue& row : *resources) {
        auto id = row.string_at("id");
        auto name = row.string_or("name", id.value_or(""));
        auto unit = row.string_or("unit", "");
        const auto mass_per_unit_kg = row.number_at("mass_per_unit_kg");
        if (const auto error = first_error(row.expect_keys({"id", "name", "unit", "mass_per_unit_kg"}), id,
                                           name, unit, mass_per_unit_kg)) {
            return std::unexpected(*error);
        }
        if (core::VoidResult added =
                updated.add_resource(ResourceDefinition{.id = std::move(*id),
                                                        .name = std::move(*name),
                                                        .unit = std::move(*unit),
                                                        .mass_per_unit_kg = *mass_per_unit_kg});
            !added) {
            return added;
        }
    }
    for (const TomlValue& row : *interfaces) {
        auto id = row.string_at("id");
        auto inputs = row.strings_or_empty("inputs");
        auto outputs = row.strings_or_empty("outputs");
        if (const auto error =
                first_error(row.expect_keys({"id", "inputs", "outputs"}), id, inputs, outputs)) {
            return std::unexpected(*error);
        }
        if (core::VoidResult added = updated.add_interface(InterfaceDefinition{
                .id = std::move(*id), .inputs = std::move(*inputs), .outputs = std::move(*outputs)});
            !added) {
            return added;
        }
    }
    for (const TomlValue& row : *parts) {
        auto part = parse_part(row);
        if (!part) {
            return std::unexpected(part.error());
        }
        if (core::VoidResult added = updated.add_part(std::move(*part)); !added) {
            return added;
        }
    }
    *this = std::move(updated);
    return {};
}

core::VoidResult PartCatalog::load_file(const std::filesystem::path& path) {
    return core::read_text_file(path)
        .and_then([&](const std::string& text) { return load_toml(text); })
        .transform_error([&](core::Error error) {
            error.context = std::format("{}: {}", path.filename().string(), error.context);
            return error;
        });
}

core::VoidResult PartCatalog::load_directory(const std::filesystem::path& directory) {
    std::vector<std::filesystem::path> files;
    std::error_code filesystem_error;
    for (std::filesystem::directory_iterator entry(directory, filesystem_error), end;
         !filesystem_error && entry != end; entry.increment(filesystem_error)) {
        if (entry->path().extension() == ".toml") {
            files.push_back(entry->path());
        }
    }
    if (filesystem_error) {
        return core::fail(ErrorCode::FileNotFound, std::format("cannot list '{}'", directory.string()));
    }
    std::ranges::sort(files);
    for (const std::filesystem::path& file : files) {
        if (core::VoidResult loaded = load_file(file); !loaded) {
            return loaded;
        }
    }
    return {};
}

} // namespace helios::vessel
