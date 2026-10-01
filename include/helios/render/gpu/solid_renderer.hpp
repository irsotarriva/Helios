#ifndef HELIOS_RENDER_GPU_SOLID_RENDERER_HPP
#define HELIOS_RENDER_GPU_SOLID_RENDERER_HPP

#include "helios/core/error.hpp"
#include "helios/render/flight_geometry.hpp"

#include <bgfx/bgfx.h>

namespace helios::render::gpu {

// Draws the solids of the flight view (parts, the cockpit, the ground) into a view that
// MapRenderer::draw has already set up with the same camera: both use reversed-Z depth, so
// they sort against each other. Create after bgfx::init; destroy before bgfx::shutdown.
class SolidRenderer {
public:
    [[nodiscard]] static core::Result<SolidRenderer> make();

    SolidRenderer(const SolidRenderer&) = delete;
    SolidRenderer& operator=(const SolidRenderer&) = delete;
    SolidRenderer(SolidRenderer&& other) noexcept;
    SolidRenderer& operator=(SolidRenderer&& other) noexcept;
    ~SolidRenderer();

    void draw(const FlightGeometry& geometry, bgfx::ViewId view) const;

private:
    struct Handles {
        bgfx::ProgramHandle program = BGFX_INVALID_HANDLE;
        bgfx::VertexBufferHandle box_vertices = BGFX_INVALID_HANDLE;
        bgfx::IndexBufferHandle box_indices = BGFX_INVALID_HANDLE;
        bgfx::VertexBufferHandle cylinder_vertices = BGFX_INVALID_HANDLE;
        bgfx::IndexBufferHandle cylinder_indices = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle colour_uniform = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle sun_uniform = BGFX_INVALID_HANDLE;
        bgfx::UniformHandle params_uniform = BGFX_INVALID_HANDLE;
    };

    SolidRenderer() = default;
    void release() noexcept;

    Handles handles_;
    bgfx::VertexLayout layout_;
};

} // namespace helios::render::gpu

#endif // HELIOS_RENDER_GPU_SOLID_RENDERER_HPP
