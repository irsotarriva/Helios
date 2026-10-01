#include "helios/render/gpu/solid_renderer.hpp"

#include "helios/render/gpu/embedded_shaders.hpp"
#include "helios/render/mesh.hpp"

#include <array>
#include <cstring>
#include <utility>

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
    return renderer;
}

SolidRenderer::SolidRenderer(SolidRenderer&& other) noexcept
    : handles_(std::exchange(other.handles_, Handles{})), layout_(other.layout_) {
}

SolidRenderer& SolidRenderer::operator=(SolidRenderer&& other) noexcept {
    if (this != &other) {
        release();
        handles_ = std::exchange(other.handles_, Handles{});
        layout_ = other.layout_;
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
}

void SolidRenderer::draw(const FlightGeometry& geometry, bgfx::ViewId view) const {
    const std::array<float, 4> sun{geometry.sun_direction.x, geometry.sun_direction.y,
                                   geometry.sun_direction.z, 0.0F};
    const auto submit = [&](std::uint32_t abgr, float cabin_light, float grid, float fill) {
        const std::array<float, 4> colour = unpack_abgr(abgr);
        const std::array<float, 4> params{cabin_light, grid, fill, 0.0F};
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
            submit(geometry.terrain->abgr, 0.0F, k_ground_grid, k_ground_fill);
        }
    }

    for (const SolidInstance& solid : geometry.solids) {
        const bool is_box = solid.shape == SolidShape::Box;
        bgfx::setTransform(solid.model.data());
        bgfx::setVertexBuffer(0, is_box ? handles_.box_vertices : handles_.cylinder_vertices);
        bgfx::setIndexBuffer(is_box ? handles_.box_indices : handles_.cylinder_indices);
        submit(solid.abgr, solid.cabin_light, 0.0F, k_solid_fill);
    }
}

} // namespace helios::render::gpu
