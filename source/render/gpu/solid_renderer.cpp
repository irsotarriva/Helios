#include "helios/render/gpu/solid_renderer.hpp"

#include "helios/core/logging.hpp"
#include "helios/render/gpu/embedded_shaders.hpp"
#include "helios/render/mesh.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <span>
#include <stb_image.h>
#include <utility>
#include <vector>

namespace helios::render::gpu {

namespace {

constexpr int k_cylinder_segments = 32;
constexpr float k_ground_grid = 1.0F;
// Light on faces turned away from the star. The ground has almost none; a vessel gets some, as
// if from the ground and its own parts, or its night side would be a black outline.
constexpr float k_ground_fill = 0.04F;
constexpr float k_solid_fill = 0.22F;
static_assert(sizeof(MeshVertex) == 32, "MeshVertex must match the bgfx solid layout");

constexpr std::uint64_t k_solid_state = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_WRITE_Z
                                        | BGFX_STATE_DEPTH_TEST_GREATER | BGFX_STATE_CULL_CW
                                        | BGFX_STATE_MSAA;

constexpr Matrix4f k_identity{1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
                              0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F};

[[nodiscard]] std::array<float, 4> unpack_abgr(std::uint32_t abgr) noexcept {
    constexpr float k_scale = 1.0F / 255.0F;
    return {static_cast<float>(abgr & 0xFFU) * k_scale, static_cast<float>((abgr >> 8U) & 0xFFU) * k_scale,
            static_cast<float>((abgr >> 16U) & 0xFFU) * k_scale,
            static_cast<float>((abgr >> 24U) & 0xFFU) * k_scale};
}

// Pictures are sampled smoothly between their pixels and their levels, and sharply along a
// panel seen edge on.
constexpr std::uint64_t k_texture_flags = BGFX_SAMPLER_MIN_ANISOTROPIC | BGFX_SAMPLER_MAG_ANISOTROPIC;

// A picture's pixels (red, green, blue and alpha, 8 bits each, from its top row) followed by
// each smaller level down to one pixel, every one the average of four of the one before.
constexpr std::size_t k_channels = 4;

[[nodiscard]] std::vector<std::uint8_t> with_levels(std::span<const std::uint8_t> pixels, std::uint32_t width,
                                                    std::uint32_t height) {
    std::vector<std::uint8_t> levels(pixels.begin(), pixels.end());
    std::size_t from = 0;
    while (width > 1 || height > 1) {
        const std::uint32_t next_width = std::max(width / 2, 1U);
        const std::uint32_t next_height = std::max(height / 2, 1U);
        const std::size_t to = levels.size();
        levels.resize(to + (std::size_t{next_width} * next_height * k_channels));
        for (std::uint32_t row = 0; row < next_height; ++row) {
            for (std::uint32_t column = 0; column < next_width; ++column) {
                const std::uint32_t row_low = std::min(2 * row, height - 1);
                const std::uint32_t row_high = std::min((2 * row) + 1, height - 1);
                const std::uint32_t column_low = std::min(2 * column, width - 1);
                const std::uint32_t column_high = std::min((2 * column) + 1, width - 1);
                for (std::size_t channel = 0; channel < k_channels; ++channel) {
                    const auto at = [&](std::uint32_t y, std::uint32_t x) {
                        return static_cast<unsigned>(
                            levels[from + ((std::size_t{y} * width + x) * k_channels) + channel]);
                    };
                    const unsigned sum = at(row_low, column_low) + at(row_low, column_high)
                                         + at(row_high, column_low) + at(row_high, column_high);
                    levels[to + ((std::size_t{row} * next_width + column) * k_channels) + channel] =
                        static_cast<std::uint8_t>((sum + 2U) / 4U);
                }
            }
        }
        from = to;
        width = next_width;
        height = next_height;
    }
    return levels;
}

struct PixelsDeleter {
    void operator()(stbi_uc* pixels) const noexcept { stbi_image_free(pixels); }
};

// A model's picture on the GPU, with its levels; invalid if it cannot be decoded or is larger
// than a texture may be.
[[nodiscard]] bgfx::TextureHandle upload(const ModelTexture& texture) {
    constexpr int k_largest_px = 16384;
    int width = 0;
    int height = 0;
    int channels = 0;
    const std::unique_ptr<stbi_uc, PixelsDeleter> pixels(
        stbi_load_from_memory(texture.encoded.data(), static_cast<int>(texture.encoded.size()), &width,
                              &height, &channels, static_cast<int>(k_channels)));
    if (pixels == nullptr || width <= 0 || height <= 0 || width > k_largest_px || height > k_largest_px) {
        LOG_WARN("a model's {} picture cannot be decoded; drawn without it", texture.mime)
            .tag("subsystem", "render");
        return BGFX_INVALID_HANDLE;
    }
    const auto columns = static_cast<std::uint32_t>(width);
    const auto rows = static_cast<std::uint32_t>(height);
    const std::vector<std::uint8_t> levels =
        with_levels(std::span(pixels.get(), std::size_t{columns} * rows * k_channels), columns, rows);
    return bgfx::createTexture2D(static_cast<std::uint16_t>(columns), static_cast<std::uint16_t>(rows), true,
                                 1, bgfx::TextureFormat::RGBA8, k_texture_flags,
                                 bgfx::copy(levels.data(), static_cast<std::uint32_t>(levels.size())));
}

template <typename Handle>
void destroy_if_valid(Handle& handle) noexcept {
    if (bgfx::isValid(handle)) {
        bgfx::destroy(handle);
        handle = BGFX_INVALID_HANDLE;
    }
}

[[nodiscard]] bgfx::VertexBufferHandle vertex_buffer(const Mesh& mesh, const bgfx::VertexLayout& layout) {
    return bgfx::createVertexBuffer(
        bgfx::copy(mesh.vertices.data(),
                   static_cast<std::uint32_t>(mesh.vertices.size() * sizeof(MeshVertex))),
        layout);
}

[[nodiscard]] bgfx::IndexBufferHandle index_buffer(const Mesh& mesh) {
    return bgfx::createIndexBuffer(bgfx::copy(
        mesh.indices.data(), static_cast<std::uint32_t>(mesh.indices.size() * sizeof(std::uint16_t))));
}

} // namespace

core::Result<SolidRenderer> SolidRenderer::make() {
    SolidRenderer renderer;
    auto program = load_program("vs_solid", "fs_solid");
    if (!program) {
        return std::unexpected(program.error());
    }
    renderer.handles_.program = *program;
    auto text_program = load_program("vs_line", "fs_line");
    if (!text_program) {
        return std::unexpected(text_program.error());
    }
    renderer.handles_.text_program = *text_program;
    renderer.text_layout_.begin()
        .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
        .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
        .end();
    renderer.layout_.begin()
        .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
        .add(bgfx::Attrib::Normal, 3, bgfx::AttribType::Float)
        .add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float)
        .end();
    const Mesh box = make_box();
    const Mesh cylinder = make_cylinder(k_cylinder_segments);
    renderer.handles_.box_vertices = vertex_buffer(box, renderer.layout_);
    renderer.handles_.box_indices = index_buffer(box);
    renderer.handles_.cylinder_vertices = vertex_buffer(cylinder, renderer.layout_);
    renderer.handles_.cylinder_indices = index_buffer(cylinder);
    renderer.handles_.colour_uniform = bgfx::createUniform("u_color", bgfx::UniformType::Vec4);
    renderer.handles_.sun_uniform = bgfx::createUniform("u_sun", bgfx::UniformType::Vec4);
    renderer.handles_.params_uniform = bgfx::createUniform("u_params", bgfx::UniformType::Vec4);
    renderer.handles_.albedo_sampler = bgfx::createUniform("s_albedo", bgfx::UniformType::Sampler);
    constexpr std::array<std::uint8_t, 4> k_white{255, 255, 255, 255};
    renderer.handles_.white =
        bgfx::createTexture2D(1, 1, false, 1, bgfx::TextureFormat::RGBA8, BGFX_SAMPLER_NONE,
                              bgfx::copy(k_white.data(), static_cast<std::uint32_t>(k_white.size())));
    return renderer;
}

SolidRenderer::SolidRenderer(SolidRenderer&& other) noexcept
    : handles_(std::exchange(other.handles_, Handles{})),
      model_meshes_(std::exchange(other.model_meshes_, {})),
      model_textures_(std::exchange(other.model_textures_, {})), layout_(other.layout_),
      text_layout_(other.text_layout_) {
}

SolidRenderer& SolidRenderer::operator=(SolidRenderer&& other) noexcept {
    if (this != &other) {
        release();
        handles_ = std::exchange(other.handles_, Handles{});
        model_meshes_ = std::exchange(other.model_meshes_, {});
        model_textures_ = std::exchange(other.model_textures_, {});
        layout_ = other.layout_;
        text_layout_ = other.text_layout_;
    }
    return *this;
}

SolidRenderer::~SolidRenderer() {
    release();
}

void SolidRenderer::release() noexcept {
    destroy_if_valid(handles_.program);
    destroy_if_valid(handles_.box_vertices);
    destroy_if_valid(handles_.box_indices);
    destroy_if_valid(handles_.cylinder_vertices);
    destroy_if_valid(handles_.cylinder_indices);
    destroy_if_valid(handles_.colour_uniform);
    destroy_if_valid(handles_.sun_uniform);
    destroy_if_valid(handles_.params_uniform);
    destroy_if_valid(handles_.text_program);
    destroy_if_valid(handles_.albedo_sampler);
    destroy_if_valid(handles_.white);
    for (MeshBuffers& mesh : model_meshes_) {
        destroy_if_valid(mesh.vertices);
        destroy_if_valid(mesh.indices);
    }
    model_meshes_.clear();
    for (bgfx::TextureHandle& texture : model_textures_) {
        destroy_if_valid(texture);
    }
    model_textures_.clear();
}

void SolidRenderer::add_models(const ModelLibrary& library) {
    const std::span<const ModelMesh> meshes = library.meshes();
    for (std::size_t index = model_meshes_.size(); index < meshes.size(); ++index) {
        const ModelMesh& mesh = meshes[index];
        MeshBuffers buffers;
        if (!mesh.vertices.empty() && !mesh.indices.empty()) {
            buffers.vertices = bgfx::createVertexBuffer(
                bgfx::copy(mesh.vertices.data(),
                           static_cast<std::uint32_t>(mesh.vertices.size() * sizeof(MeshVertex))),
                layout_);
            buffers.indices = bgfx::createIndexBuffer(
                bgfx::copy(mesh.indices.data(),
                           static_cast<std::uint32_t>(mesh.indices.size() * sizeof(std::uint32_t))),
                BGFX_BUFFER_INDEX32);
        }
        model_meshes_.push_back(buffers);
    }
    const std::span<const ModelTexture> textures = library.textures();
    for (std::size_t index = model_textures_.size(); index < textures.size(); ++index) {
        model_textures_.push_back(upload(textures[index]));
    }
}

void SolidRenderer::draw(const FlightGeometry& geometry, bgfx::ViewId view) const {
    const std::array<float, 4> sun{geometry.sun_direction.x, geometry.sun_direction.y,
                                   geometry.sun_direction.z, 0.0F};
    const auto submit = [&](std::uint32_t abgr, float cabin_light, float grid, float fill,
                            bgfx::TextureHandle texture) {
        const std::array<float, 4> colour = unpack_abgr(abgr);
        const std::array<float, 4> params{cabin_light, grid, fill, 0.0F};
        bgfx::setTexture(0, handles_.albedo_sampler, texture);
        bgfx::setUniform(handles_.colour_uniform, colour.data());
        bgfx::setUniform(handles_.sun_uniform, sun.data());
        bgfx::setUniform(handles_.params_uniform, params.data());
        bgfx::setState(k_solid_state);
        bgfx::submit(view, handles_.program);
    };

    if (geometry.terrain.has_value()) {
        const Mesh& mesh = geometry.terrain->mesh;
        const auto vertex_count = static_cast<std::uint32_t>(mesh.vertices.size());
        const auto index_count = static_cast<std::uint32_t>(mesh.indices.size());
        // Rationale: the patch is rebuilt every frame around the camera, so it lives in the
        // frame's transient buffers. If they are full the ground is missing for a frame.
        if (bgfx::getAvailTransientVertexBuffer(vertex_count, layout_) == vertex_count
            && bgfx::getAvailTransientIndexBuffer(index_count) == index_count) {
            bgfx::TransientVertexBuffer vertices{};
            bgfx::TransientIndexBuffer indices{};
            bgfx::allocTransientVertexBuffer(&vertices, vertex_count, layout_);
            bgfx::allocTransientIndexBuffer(&indices, index_count);
            std::memcpy(vertices.data, mesh.vertices.data(), mesh.vertices.size() * sizeof(MeshVertex));
            std::memcpy(indices.data, mesh.indices.data(), mesh.indices.size() * sizeof(std::uint16_t));
            bgfx::setTransform(k_identity.data());
            bgfx::setVertexBuffer(0, &vertices);
            bgfx::setIndexBuffer(&indices);
            submit(geometry.terrain->abgr, 0.0F, k_ground_grid, k_ground_fill, handles_.white);
        }
    }

    for (const SolidInstance& solid : geometry.solids) {
        const bool is_box = solid.shape == SolidShape::Box;
        bgfx::setTransform(solid.model.data());
        bgfx::setVertexBuffer(0, is_box ? handles_.box_vertices : handles_.cylinder_vertices);
        bgfx::setIndexBuffer(is_box ? handles_.box_indices : handles_.cylinder_indices);
        submit(solid.abgr, solid.cabin_light, 0.0F, k_solid_fill, handles_.white);
    }
    for (const ModelInstance& instance : geometry.models) {
        if (instance.mesh >= model_meshes_.size() || !bgfx::isValid(model_meshes_[instance.mesh].vertices)) {
            continue;
        }
        bgfx::setTransform(instance.model.data());
        bgfx::setVertexBuffer(0, model_meshes_[instance.mesh].vertices);
        bgfx::setIndexBuffer(model_meshes_[instance.mesh].indices);
        bgfx::TextureHandle texture = handles_.white;
        if (instance.texture.has_value() && *instance.texture < model_textures_.size()
            && bgfx::isValid(model_textures_[*instance.texture])) {
            texture = model_textures_[*instance.texture];
        }
        submit(instance.abgr, instance.cabin_light, 0.0F, k_solid_fill, texture);
    }
    draw_text(geometry, view);
}

void SolidRenderer::draw_text(const FlightGeometry& geometry, bgfx::ViewId view) const {
    const auto count = static_cast<std::uint32_t>(geometry.text.size());
    // Rationale: like the ground, rebuilt every frame; if the transient buffers are full the
    // lettering is missing for a frame.
    if (count == 0U || bgfx::getAvailTransientVertexBuffer(count, text_layout_) != count) {
        return;
    }
    bgfx::TransientVertexBuffer vertices{};
    bgfx::allocTransientVertexBuffer(&vertices, count, text_layout_);
    std::memcpy(vertices.data, geometry.text.data(), geometry.text.size() * sizeof(LineVertex));
    bgfx::setTransform(k_identity.data());
    bgfx::setVertexBuffer(0, &vertices);
    // Not culled: the lettering is seen from the front, and the panel hides it from behind.
    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_WRITE_Z
                   | BGFX_STATE_DEPTH_TEST_GREATER | BGFX_STATE_MSAA);
    bgfx::submit(view, handles_.text_program);
}

} // namespace helios::render::gpu
