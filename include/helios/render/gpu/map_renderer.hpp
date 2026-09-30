#ifndef HELIOS_RENDER_GPU_MAP_RENDERER_HPP
#define HELIOS_RENDER_GPU_MAP_RENDERER_HPP

#include "helios/core/error.hpp"
#include "helios/render/camera.hpp"
#include "helios/render/scene_geometry.hpp"

#include <bgfx/bgfx.h>
#include <cstdint>

namespace helios::render::gpu {

// Where one view is drawn: a bgfx view and a rectangle of the back buffer (or of an eye's
// swap-chain image, for VR).
struct ViewTarget {
    bgfx::ViewId view_id = 0;
    std::int16_t x = 0;
    std::int16_t y = 0;
    std::uint16_t width = 0;
    std::uint16_t height = 0;
};

// The camera of one view. VR renders the same FrameGeometry (built for the head) once per eye,
// with each eye's view (including its small offset from the head) and projection.
struct ViewCamera {
    Matrix4f view;
    Matrix4f projection;
};

// Draws the map view: lit spheres for bodies and orbit / trajectory lines, with reversed-Z
// depth (see reversed_infinite_projection). Create after bgfx::init; destroy before
// bgfx::shutdown.
class MapRenderer {
public:
    [[nodiscard]] static core::Result<MapRenderer> make();

    MapRenderer(const MapRenderer&) = delete;
    MapRenderer& operator=(const MapRenderer&) = delete;
    MapRenderer(MapRenderer&& other) noexcept;
    MapRenderer& operator=(MapRenderer&& other) noexcept;
    ~MapRenderer();

    // Clear colour 0xRRGGBBAA.
    void draw(const FrameGeometry& geometry, const ViewCamera& camera, const ViewTarget& target,
              std::uint32_t clear_rgba = 0x05070cffU) const;

private:
    MapRenderer() = default;
    void release() noexcept;

    bgfx::ProgramHandle body_program_ = BGFX_INVALID_HANDLE;
    bgfx::ProgramHandle line_program_ = BGFX_INVALID_HANDLE;
    bgfx::VertexBufferHandle sphere_vertices_ = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle sphere_indices_ = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle colour_uniform_ = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle sun_uniform_ = BGFX_INVALID_HANDLE;
    bgfx::VertexLayout line_layout_;
};

} // namespace helios::render::gpu

#endif // HELIOS_RENDER_GPU_MAP_RENDERER_HPP
