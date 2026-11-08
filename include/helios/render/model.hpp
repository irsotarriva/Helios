#ifndef HELIOS_RENDER_MODEL_HPP
#define HELIOS_RENDER_MODEL_HPP

#include "helios/core/error.hpp"
#include "helios/math/vector3.hpp"
#include "helios/render/mesh.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// Models a look can be drawn with instead of a built-in shape (BRIEFING D44): glTF files in
// data/models, each listed with its licence in data/models/LICENCES.toml. Like the rest of
// render/*.cpp this is GPU-free: a file goes in, meshes in the model's own axes come out.
namespace helios::render {

// Triangles counter-clockwise seen from outside, as Mesh, with indices for larger meshes.
struct ModelMesh {
    std::vector<MeshVertex> vertices;
    std::vector<std::uint32_t> indices;
};

// What colours a piece of a model: its own material, or the look's colours (data/models/README.md).
enum class ModelPaint : std::uint8_t {
    Own,    // the material's base colour
    Colour, // the look's `colour`
    Accent, // the look's `accent`: grey while the control is dead
    Paint,  // the look's `paint`
    Lamp,   // `colour` when out, `accent` lit, between them for the states between
};

// One mesh of a model, already placed in the model's axes (x right, y up, z out of the panel).
struct ModelPiece {
    std::size_t mesh = 0;                // its index in the library's meshes
    math::Vector3 colour{0.8, 0.8, 0.8}; // its own, as displayed: red, green, blue from 0 to 1
    ModelPaint paint = ModelPaint::Own;
    bool moving = false; // under the node called `moving`: moves as the control does
};

struct Model {
    std::string path; // relative to data/models, as a look names it
    std::vector<ModelPiece> pieces;
    math::Vector3 pivot_m; // what `moving` turns about: that node's origin, in the model's axes
};

// Who made a model and under which terms it may be used, as LICENCES.toml lists it.
struct ModelLicence {
    std::string file;
    std::string title;
    std::string author;
    std::string licence; // an SPDX identifier, e.g. "CC-BY-4.0"
    std::string source;  // where it came from: a URL, or the script that made it
};

// Reads one glTF file (.glb or .gltf) into a model whose meshes are appended to `meshes`.
// Only triangles are read; a material's base colour is its own colour, and its name may give it
// the look's (`colour`, `accent`, `paint`, `lamp`). Textures are not read yet.
[[nodiscard]] core::Result<Model> load_model(const std::filesystem::path& file, std::string path,
                                             std::vector<ModelMesh>& meshes);

// The licences of the models in a directory: its LICENCES.toml.
[[nodiscard]] core::Result<std::vector<ModelLicence>> load_licences(const std::filesystem::path& directory);

// The models looks may name, by path. Only a model listed with its licence is read.
class ModelLibrary {
public:
    // Every model LICENCES.toml in `directory` lists; fails if one cannot be read.
    [[nodiscard]] static core::Result<ModelLibrary> load(const std::filesystem::path& directory);

    // None for a path that names no model here.
    [[nodiscard]] const Model* find(std::string_view path) const noexcept;
    [[nodiscard]] std::span<const Model> models() const noexcept { return models_; }
    // Every model's meshes, in the order they were read: a renderer uploads them once.
    [[nodiscard]] std::span<const ModelMesh> meshes() const noexcept { return meshes_; }
    [[nodiscard]] std::span<const ModelLicence> licences() const noexcept { return licences_; }

private:
    std::vector<Model> models_;
    std::vector<ModelMesh> meshes_;
    std::vector<ModelLicence> licences_;
};

} // namespace helios::render

#endif // HELIOS_RENDER_MODEL_HPP
