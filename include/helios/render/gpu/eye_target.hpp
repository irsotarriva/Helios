#ifndef HELIOS_RENDER_GPU_EYE_TARGET_HPP
#define HELIOS_RENDER_GPU_EYE_TARGET_HPP

#include "helios/core/error.hpp"

#include <bgfx/bgfx.h>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace helios::render::gpu {

// The images one eye of a headset is drawn into (BRIEFING D29): textures that the headset's
// runtime made on the renderer's own graphics device, wrapped so that a view can be drawn into
// them like into any other target, with a depth buffer of ours. The runtime owns the textures;
// they must outlive this. Create after bgfx::init; destroy before bgfx::shutdown.
class EyeTarget {
public:
    struct Images {
        std::uint16_t width_px = 0;
        std::uint16_t height_px = 0;
        bgfx::TextureFormat::Enum format = bgfx::TextureFormat::RGBA8;
        std::span<const std::uintptr_t> textures; // native textures of the renderer in use
    };

    [[nodiscard]] static core::Result<EyeTarget> make(const Images& images);

    EyeTarget(const EyeTarget&) = delete;
    EyeTarget& operator=(const EyeTarget&) = delete;
    EyeTarget(EyeTarget&& other) noexcept;
    EyeTarget& operator=(EyeTarget&& other) noexcept;
    ~EyeTarget();

    [[nodiscard]] std::uint16_t width_px() const noexcept { return width_px_; }
    [[nodiscard]] std::uint16_t height_px() const noexcept { return height_px_; }
    [[nodiscard]] std::size_t image_count() const noexcept { return frame_buffers_.size(); }

    // Points a view at one of the images; what is submitted to the view is then drawn into it.
    void bind(bgfx::ViewId view, std::size_t image) const;

private:
    EyeTarget() = default;
    void release() noexcept;

    std::uint16_t width_px_ = 0;
    std::uint16_t height_px_ = 0;
    bgfx::TextureHandle depth_ = BGFX_INVALID_HANDLE;
    std::vector<bgfx::TextureHandle> colours_;
    std::vector<bgfx::FrameBufferHandle> frame_buffers_;
};

} // namespace helios::render::gpu

#endif // HELIOS_RENDER_GPU_EYE_TARGET_HPP
