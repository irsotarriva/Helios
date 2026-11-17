#ifndef HELIOS_RENDER_GPU_SCREEN_RENDERER_HPP
#define HELIOS_RENDER_GPU_SCREEN_RENDERER_HPP

#include "helios/core/error.hpp"
#include "helios/render/flight_geometry.hpp"

#include <bgfx/bgfx.h>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace helios::render::gpu {

// The cockpit's screens (BRIEFING D46, §12.1 rule 3): each screen's picture is drawn into a
// texture of its own resolution, and the texture is put on its glass in the world, so it is
// seen from wherever the eye is and in a headset as on a flat screen. Create after bgfx::init;
// destroy before bgfx::shutdown.
class ScreenRenderer {
public:
    // The most screens painted in a frame; the views they take, from the first one given.
    static constexpr std::size_t k_most_screens = 8;

    [[nodiscard]] static core::Result<ScreenRenderer> make();

    ScreenRenderer(const ScreenRenderer&) = delete;
    ScreenRenderer& operator=(const ScreenRenderer&) = delete;
    ScreenRenderer(ScreenRenderer&& other) noexcept;
    ScreenRenderer& operator=(ScreenRenderer&& other) noexcept;
    ~ScreenRenderer();

    // Draws each screen's picture into its texture, the first in `first_view` and each next one
    // in the view after; those views have to come before the ones the glass is drawn in
    // (bgfx::setViewOrder). Textures of screens no longer painted are let go.
    void paint(std::span<const ScreenSurface> screens, bgfx::ViewId first_view);

    // Draws the glass of the screens last painted into `view`, which MapRenderer::draw has set
    // up for the camera the screens' corners were given for.
    void draw(std::span<const ScreenSurface> screens, bgfx::ViewId view) const;

private:
    struct Target {
        std::size_t part = 0;
        std::size_t screen = 0;
        std::uint16_t width_px = 0;
        std::uint16_t height_px = 0;
        bgfx::FrameBufferHandle frame = BGFX_INVALID_HANDLE;
        bgfx::TextureHandle texture = BGFX_INVALID_HANDLE;
    };

    ScreenRenderer() = default;
    void release() noexcept;
    [[nodiscard]] const Target* target_of(const ScreenSurface& surface) const noexcept;

    bgfx::ProgramHandle ink_program_ = BGFX_INVALID_HANDLE;   // the pictures, coloured per vertex
    bgfx::ProgramHandle glass_program_ = BGFX_INVALID_HANDLE; // the glass, textured
    bgfx::UniformHandle sampler_ = BGFX_INVALID_HANDLE;
    bgfx::VertexLayout ink_layout_;
    bgfx::VertexLayout glass_layout_;
    std::vector<Target> targets_;
};

} // namespace helios::render::gpu

#endif // HELIOS_RENDER_GPU_SCREEN_RENDERER_HPP
