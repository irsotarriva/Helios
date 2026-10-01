#include "helios/render/gpu/map_renderer.hpp"

#include "helios/render/gpu/embedded_shaders.hpp"
#include "helios/render/mesh.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <span>
#include <utility>

namespace helios::render::gpu {

namespace {

constexpr int k_sphere_subdivisions = 4;
static_assert(sizeof(LineVertex) == 16, "LineVertex must match the bgfx line layout");
static_assert(sizeof(Float3) == 12, "Float3 must be tightly packed");

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

} // namespace

core::Result<MapRenderer> MapRenderer::make() {
    MapRenderer renderer;
    auto body_program = load_program("vs_body", "fs_body");
    if (!body_program) {
        return std::unexpected(body_program.error());
    }
    renderer.body_program_ = *body_program;
    auto line_program = load_program("vs_line", "fs_line");
    if (!line_program) {
        return std::unexpected(line_program.error());
    }
    renderer.line_program_ = *line_program;

    const SphereMesh sphere = make_icosphere(k_sphere_subdivisions);
    bgfx::VertexLayout sphere_layout;
    sphere_layout.begin().add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float).end();
    renderer.sphere_vertices_ = bgfx::createVertexBuffer(
        bgfx::copy(sphere.vertices.data(),
                   static_cast<std::uint32_t>(sphere.vertices.size() * sizeof(Float3))),
        sphere_layout);
    renderer.sphere_indices_ = bgfx::createIndexBuffer(bgfx::copy(
        sphere.indices.data(), static_cast<std::uint32_t>(sphere.indices.size() * sizeof(std::uint16_t))));
    renderer.colour_uniform_ = bgfx::createUniform("u_color", bgfx::UniformType::Vec4);
    renderer.sun_uniform_ = bgfx::createUniform("u_sun", bgfx::UniformType::Vec4);
    renderer.line_layout_.begin()
        .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
        .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
        .end();
    return renderer;
}

MapRenderer::MapRenderer(MapRenderer&& other) noexcept
    : body_program_(std::exchange(other.body_program_, BGFX_INVALID_HANDLE)),
      line_program_(std::exchange(other.line_program_, BGFX_INVALID_HANDLE)),
      sphere_vertices_(std::exchange(other.sphere_vertices_, BGFX_INVALID_HANDLE)),
      sphere_indices_(std::exchange(other.sphere_indices_, BGFX_INVALID_HANDLE)),
      colour_uniform_(std::exchange(other.colour_uniform_, BGFX_INVALID_HANDLE)),
      sun_uniform_(std::exchange(other.sun_uniform_, BGFX_INVALID_HANDLE)), line_layout_(other.line_layout_) {
}

MapRenderer& MapRenderer::operator=(MapRenderer&& other) noexcept {
    if (this != &other) {
        release();
        body_program_ = std::exchange(other.body_program_, BGFX_INVALID_HANDLE);
        line_program_ = std::exchange(other.line_program_, BGFX_INVALID_HANDLE);
        sphere_vertices_ = std::exchange(other.sphere_vertices_, BGFX_INVALID_HANDLE);
        sphere_indices_ = std::exchange(other.sphere_indices_, BGFX_INVALID_HANDLE);
        colour_uniform_ = std::exchange(other.colour_uniform_, BGFX_INVALID_HANDLE);
        sun_uniform_ = std::exchange(other.sun_uniform_, BGFX_INVALID_HANDLE);
        line_layout_ = other.line_layout_;
    }
    return *this;
}

MapRenderer::~MapRenderer() {
    release();
}

void MapRenderer::release() noexcept {
    destroy_if_valid(body_program_);
    destroy_if_valid(line_program_);
    destroy_if_valid(sphere_vertices_);
    destroy_if_valid(sphere_indices_);
    destroy_if_valid(colour_uniform_);
    destroy_if_valid(sun_uniform_);
}

void MapRenderer::draw(const FrameGeometry& geometry, const ViewCamera& camera, const ViewTarget& target,
                       std::uint32_t clear_rgba) const {
    const bgfx::ViewId view = target.view_id;
    // Reversed depth: clear to 0 (infinitely far), keep the greater depth.
    bgfx::setViewClear(view, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH, clear_rgba, 0.0F, 0);
    bgfx::setViewRect(view, target.x, target.y, target.width, target.height);
    bgfx::setViewTransform(view, camera.view.data(), camera.projection.data());
    bgfx::touch(view);

    constexpr std::uint64_t k_body_state = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_WRITE_Z
                                           | BGFX_STATE_DEPTH_TEST_GREATER | BGFX_STATE_CULL_CW
                                           | BGFX_STATE_MSAA;
    for (const BodyInstance& body : geometry.bodies) {
        const std::array<float, 4> colour = unpack_abgr(body.abgr);
        const std::array<float, 4> sun{body.sun_direction.x, body.sun_direction.y, body.sun_direction.z,
                                       body.emissive ? 1.0F : 0.0F};
        bgfx::setTransform(body.model.data());
        bgfx::setUniform(colour_uniform_, colour.data());
        bgfx::setUniform(sun_uniform_, sun.data());
        bgfx::setVertexBuffer(0, sphere_vertices_);
        bgfx::setIndexBuffer(sphere_indices_);
        bgfx::setState(k_body_state);
        bgfx::submit(view, body_program_);
    }

    constexpr std::uint64_t k_line_state =
        BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_DEPTH_TEST_GREATER | BGFX_STATE_PT_LINES
        | BGFX_STATE_LINEAA
        | BGFX_STATE_BLEND_FUNC(BGFX_STATE_BLEND_SRC_ALPHA, BGFX_STATE_BLEND_INV_SRC_ALPHA) | BGFX_STATE_MSAA;
    std::size_t first = 0;
    while (first < geometry.lines.size()) {
        const auto wanted = static_cast<std::uint32_t>(geometry.lines.size() - first);
        std::uint32_t count = std::min(wanted, bgfx::getAvailTransientVertexBuffer(wanted, line_layout_));
        count -= count % 2; // whole segments only
        if (count == 0) {
            break; // out of transient memory this frame; the remaining lines are dropped
        }
        bgfx::TransientVertexBuffer buffer{};
        bgfx::allocTransientVertexBuffer(&buffer, count, line_layout_);
        const std::span<const LineVertex> chunk = std::span(geometry.lines).subspan(first, count);
        std::memcpy(buffer.data, chunk.data(), chunk.size_bytes());
        bgfx::setVertexBuffer(0, &buffer);
        bgfx::setState(k_line_state);
        bgfx::submit(view, line_program_);
        first += count;
    }
}

} // namespace helios::render::gpu
