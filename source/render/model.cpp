#include "helios/render/model.hpp"

#include "helios/core/parse.hpp"
#include "helios/core/toml.hpp"
#include "helios/vessel/controls.hpp"

#include <algorithm>
#include <array>
#include <cgltf.h> // its implementation is in cgltf_implementation.cpp
#include <cmath>
#include <format>
#include <iterator>
#include <memory>
#include <optional>
#include <span>
#include <utility>

namespace helios::render {

namespace {

using core::ErrorCode;
using math::Vector3;

constexpr std::string_view k_licence_file = "LICENCES.toml";
constexpr std::string_view k_moving_node = "moving";

// A node's place in the model: a 4×4 column-major matrix, as glTF gives it.
using NodeMatrix = std::array<float, 16>;

[[nodiscard]] Vector3 transform_point(const NodeMatrix& matrix, const Vector3& point) noexcept {
    const auto at = [&](std::size_t index) { return static_cast<double>(matrix.at(index)); };
    return {(at(0) * point.x) + (at(4) * point.y) + (at(8) * point.z) + at(12),
            (at(1) * point.x) + (at(5) * point.y) + (at(9) * point.z) + at(13),
            (at(2) * point.x) + (at(6) * point.y) + (at(10) * point.z) + at(14)};
}

// The cofactor matrix turns normals: it is the inverse transpose times the determinant, so it
// is right for any scale and needs no inverse.
[[nodiscard]] Vector3 transform_normal(const NodeMatrix& matrix, const Vector3& normal) noexcept {
    const auto at = [&](std::size_t column, std::size_t row) {
        return static_cast<double>(matrix.at((column * 4) + row));
    };
    const Vector3 x{at(0, 0), at(0, 1), at(0, 2)};
    const Vector3 y{at(1, 0), at(1, 1), at(1, 2)};
    const Vector3 z{at(2, 0), at(2, 1), at(2, 2)};
    const Vector3 turned =
        (normal.x * math::cross(y, z)) + (normal.y * math::cross(z, x)) + (normal.z * math::cross(x, y));
    const double length = math::norm(turned);
    return length > 0.0 ? turned / length : normal;
}

[[nodiscard]] double determinant(const NodeMatrix& matrix) noexcept {
    const auto column = [&](std::size_t index) {
        return Vector3{matrix.at(index * 4), matrix.at((index * 4) + 1), matrix.at((index * 4) + 2)};
    };
    return math::dot(column(0), math::cross(column(1), column(2)));
}

// glTF's colours are linear; the solid shader's are as displayed.
[[nodiscard]] double to_display(double linear) noexcept {
    const double clamped = std::clamp(linear, 0.0, 1.0);
    return clamped <= 0.0031308 ? 12.92 * clamped : (1.055 * std::pow(clamped, 1.0 / 2.4)) - 0.055;
}

[[nodiscard]] ModelPaint paint_of(const cgltf_material* material) noexcept {
    if (material == nullptr || material->name == nullptr) {
        return ModelPaint::Own;
    }
    const std::string_view name = material->name;
    if (name == "colour") {
        return ModelPaint::Colour;
    }
    if (name == "accent") {
        return ModelPaint::Accent;
    }
    if (name == "paint") {
        return ModelPaint::Paint;
    }
    if (name == "lamp") {
        return ModelPaint::Lamp;
    }
    return ModelPaint::Own;
}

[[nodiscard]] Vector3 colour_of(const cgltf_material* material) noexcept {
    if (material == nullptr || material->has_pbr_metallic_roughness == 0) {
        return ModelPiece{}.colour;
    }
    const std::span<const cgltf_float, 4> factor{material->pbr_metallic_roughness.base_color_factor};
    return {to_display(factor[0]), to_display(factor[1]), to_display(factor[2])};
}

[[nodiscard]] const cgltf_accessor* attribute(const cgltf_primitive& primitive,
                                              cgltf_attribute_type type) noexcept {
    for (const cgltf_attribute& entry : std::span(primitive.attributes, primitive.attributes_count)) {
        if (entry.type == type && entry.index == 0) {
            return entry.data;
        }
    }
    return nullptr;
}

[[nodiscard]] Vector3 read_vector(const cgltf_accessor* accessor, cgltf_size index) noexcept {
    std::array<cgltf_float, 3> value{};
    if (cgltf_accessor_read_float(accessor, index, value.data(), value.size()) == 0) {
        return {};
    }
    return {value[0], value[1], value[2]};
}

// Normals for a mesh that has none: each vertex's is the sum of its triangles' (weighted by
// their areas), so a curved surface looks smooth and a flat one flat.
void add_normals(ModelMesh& mesh) {
    std::vector<Vector3> sums(mesh.vertices.size());
    const auto position = [&](std::uint32_t index) {
        const Float3& at = mesh.vertices.at(index).position;
        return Vector3{at.x, at.y, at.z};
    };
    for (std::size_t first = 0; first + 2 < mesh.indices.size(); first += 3) {
        const std::array<std::uint32_t, 3> corner{mesh.indices[first], mesh.indices[first + 1],
                                                  mesh.indices[first + 2]};
        const Vector3 face =
            math::cross(position(corner[1]) - position(corner[0]), position(corner[2]) - position(corner[0]));
        for (const std::uint32_t index : corner) {
            sums.at(index) = sums.at(index) + face;
        }
    }
    for (std::size_t index = 0; index < mesh.vertices.size(); ++index) {
        const double length = math::norm(sums[index]);
        const Vector3 normal = length > 0.0 ? sums[index] / length : Vector3{0.0, 0.0, 1.0};
        mesh.vertices[index].normal = {static_cast<float>(normal.x), static_cast<float>(normal.y),
                                       static_cast<float>(normal.z)};
    }
}

// One primitive of a node's mesh, placed in the model's axes.
[[nodiscard]] core::Result<ModelMesh> read_primitive(const cgltf_primitive& primitive,
                                                     const NodeMatrix& place) {
    const cgltf_accessor* positions = attribute(primitive, cgltf_attribute_type_position);
    if (positions == nullptr) {
        return core::fail(ErrorCode::ParseFailure, "a mesh without positions");
    }
    const cgltf_accessor* normals = attribute(primitive, cgltf_attribute_type_normal);
    const cgltf_accessor* coordinates = attribute(primitive, cgltf_attribute_type_texcoord);
    ModelMesh mesh;
    mesh.vertices.reserve(positions->count);
    for (cgltf_size index = 0; index < positions->count; ++index) {
        const Vector3 at_m = transform_point(place, read_vector(positions, index));
        const Vector3 normal =
            normals != nullptr ? transform_normal(place, read_vector(normals, index)) : Vector3{};
        std::array<cgltf_float, 2> uv{};
        if (coordinates != nullptr) {
            static_cast<void>(cgltf_accessor_read_float(coordinates, index, uv.data(), uv.size()));
        }
        mesh.vertices.push_back(MeshVertex{
            .position = {static_cast<float>(at_m.x), static_cast<float>(at_m.y), static_cast<float>(at_m.z)},
            .normal = {static_cast<float>(normal.x), static_cast<float>(normal.y),
                       static_cast<float>(normal.z)},
            .u = uv[0],
            .v = uv[1]});
    }
    if (primitive.indices != nullptr) {
        mesh.indices.reserve(primitive.indices->count);
        for (cgltf_size index = 0; index < primitive.indices->count; ++index) {
            const cgltf_size vertex = cgltf_accessor_read_index(primitive.indices, index);
            if (vertex >= mesh.vertices.size()) {
                return core::fail(ErrorCode::ParseFailure, "a mesh's index beyond its vertices");
            }
            mesh.indices.push_back(static_cast<std::uint32_t>(vertex));
        }
    } else {
        for (std::size_t index = 0; index < mesh.vertices.size(); ++index) {
            mesh.indices.push_back(static_cast<std::uint32_t>(index));
        }
    }
    mesh.indices.resize(mesh.indices.size() - (mesh.indices.size() % 3));
    // A mirrored node turns its triangles inside out; put them back.
    if (determinant(place) < 0.0) {
        for (std::size_t first = 0; first + 2 < mesh.indices.size(); first += 3) {
            std::swap(mesh.indices[first + 1], mesh.indices[first + 2]);
        }
    }
    if (normals == nullptr) {
        add_normals(mesh);
    }
    return mesh;
}

[[nodiscard]] bool under_moving(const cgltf_node* node) noexcept {
    for (const cgltf_node* at = node; at != nullptr; at = at->parent) {
        if (at->name != nullptr && std::string_view{at->name} == k_moving_node) {
            return true;
        }
    }
    return false;
}

struct DataDeleter {
    void operator()(cgltf_data* data) const noexcept { cgltf_free(data); }
};

[[nodiscard]] std::string_view result_name(cgltf_result result) noexcept {
    switch (result) {
    case cgltf_result_data_too_short: return "the file is cut short";
    case cgltf_result_unknown_format: return "not a glTF file";
    case cgltf_result_invalid_json:   return "invalid JSON";
    case cgltf_result_invalid_gltf:   return "invalid glTF";
    case cgltf_result_file_not_found: return "a file it needs is missing";
    case cgltf_result_io_error:       return "cannot read it";
    case cgltf_result_out_of_memory:  return "out of memory";
    case cgltf_result_legacy_gltf:    return "glTF 1.0, not 2.0";
    default:                          return "cannot read it";
    }
}

[[nodiscard]] core::Result<std::string> licence_text(const core::TomlValue& row, std::string_view key) {
    return row.string_at(key).and_then([&](std::string text) -> core::Result<std::string> {
        if (text.empty()) {
            return core::fail(ErrorCode::ParseFailure,
                              std::format("line {}: a model's {} must not be empty", row.line, key));
        }
        return text;
    });
}

} // namespace

core::Result<Model> load_model(const std::filesystem::path& file, std::string path,
                               std::vector<ModelMesh>& meshes) {
    const std::string name = file.string();
    const auto fail = [&](std::string_view why) {
        return core::fail(ErrorCode::ParseFailure, std::format("model '{}': {}", path, why));
    };
    const cgltf_options options{};
    cgltf_data* parsed = nullptr;
    if (const cgltf_result result = cgltf_parse_file(&options, name.c_str(), &parsed);
        result != cgltf_result_success) {
        return fail(result == cgltf_result_file_not_found ? "no such file" : result_name(result));
    }
    const std::unique_ptr<cgltf_data, DataDeleter> data(parsed);
    if (const cgltf_result result = cgltf_load_buffers(&options, data.get(), name.c_str());
        result != cgltf_result_success) {
        return fail(result_name(result));
    }
    if (const cgltf_result result = cgltf_validate(data.get()); result != cgltf_result_success) {
        return fail(result_name(result));
    }

    Model model{.path = std::move(path), .pieces = {}, .pivot_m = {}};
    std::vector<ModelMesh> read;
    for (const cgltf_node& node : std::span(data->nodes, data->nodes_count)) {
        NodeMatrix place{};
        cgltf_node_transform_world(&node, place.data());
        if (node.name != nullptr && std::string_view{node.name} == k_moving_node) {
            model.pivot_m = transform_point(place, Vector3{});
        }
        if (node.mesh == nullptr) {
            continue;
        }
        for (const cgltf_primitive& piece : std::span(node.mesh->primitives, node.mesh->primitives_count)) {
            if (piece.type != cgltf_primitive_type_triangles) {
                continue; // lines and points are not drawn
            }
            auto mesh = read_primitive(piece, place);
            if (!mesh) {
                return fail(mesh.error().context);
            }
            model.pieces.push_back(ModelPiece{.mesh = meshes.size() + read.size(),
                                              .colour = colour_of(piece.material),
                                              .paint = paint_of(piece.material),
                                              .moving = under_moving(&node)});
            read.push_back(std::move(*mesh));
        }
    }
    if (model.pieces.empty()) {
        return fail("it has no triangles");
    }
    std::ranges::move(read, std::back_inserter(meshes));
    return model;
}

core::Result<std::vector<ModelLicence>> load_licences(const std::filesystem::path& directory) {
    const std::filesystem::path file = directory / k_licence_file;
    auto document =
        core::read_text_file(file).and_then([](const std::string& text) { return core::parse_toml(text); });
    if (!document) {
        return std::unexpected(document.error());
    }
    const auto rows = document->array_or_empty("model");
    if (!rows) {
        return std::unexpected(rows.error());
    }
    std::vector<ModelLicence> licences;
    for (const core::TomlValue& row : *rows) {
        auto model_file = licence_text(row, "file");
        auto title = licence_text(row, "title");
        auto author = licence_text(row, "author");
        auto licence = licence_text(row, "licence");
        auto source = licence_text(row, "source");
        if (const auto error =
                core::first_error(row.expect_keys({"file", "title", "author", "licence", "source"}),
                                  model_file, title, author, licence, source)) {
            return std::unexpected(*error);
        }
        if (!vessel::is_model_path(*model_file)) {
            return core::fail(ErrorCode::ParseFailure,
                              std::format("line {}: '{}' is not a .glb or .gltf file in this directory, as a "
                                          "relative path with forward slashes",
                                          row.line, *model_file));
        }
        if (std::ranges::find(licences, *model_file, &ModelLicence::file) != licences.end()) {
            return core::fail(ErrorCode::ParseFailure,
                              std::format("line {}: '{}' is listed twice", row.line, *model_file));
        }
        licences.push_back(ModelLicence{.file = std::move(*model_file),
                                        .title = std::move(*title),
                                        .author = std::move(*author),
                                        .licence = std::move(*licence),
                                        .source = std::move(*source)});
    }
    return licences;
}

core::Result<ModelLibrary> ModelLibrary::load(const std::filesystem::path& directory) {
    auto licences = load_licences(directory);
    if (!licences) {
        return std::unexpected(licences.error());
    }
    ModelLibrary library;
    for (const ModelLicence& licence : *licences) {
        auto model = load_model(directory / licence.file, licence.file, library.meshes_);
        if (!model) {
            return std::unexpected(model.error());
        }
        library.models_.push_back(std::move(*model));
    }
    library.licences_ = std::move(*licences);
    return library;
}

const Model* ModelLibrary::find(std::string_view path) const noexcept {
    const auto match = std::ranges::find(models_, path, &Model::path);
    return match != models_.end() ? &*match : nullptr;
}

} // namespace helios::render
