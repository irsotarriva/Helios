#include "helios/render/gpu/eye_target.hpp"

#include <array>
#include <utility>

namespace helios::render::gpu {

using core::ErrorCode;

namespace {

// Drawn into, never sampled: the runtime need not let its images be read.
constexpr std::uint64_t k_target_flags = BGFX_TEXTURE_RT_WRITE_ONLY;

template <typename Handle>
void destroy_if_valid(Handle& handle) noexcept {
    if (bgfx::isValid(handle)) {
        bgfx::destroy(handle);
        handle = BGFX_INVALID_HANDLE;
    }
}

} // namespace

core::Result<EyeTarget> EyeTarget::make(const Images& images) {
    if (images.width_px == 0 || images.height_px == 0 || images.textures.empty()) {
        return core::fail(ErrorCode::InvalidArgument, "an eye needs images to be drawn into");
    }
    EyeTarget target;
    target.width_px_ = images.width_px;
    target.height_px_ = images.height_px;
    // A 32-bit float depth buffer, as for the window: reversed depth depends on it.
    target.depth_ = bgfx::createTexture2D(images.width_px, images.height_px, false, 1,
                                          bgfx::TextureFormat::D32F, k_target_flags);
    if (!bgfx::isValid(target.depth_)) {
        return core::fail(ErrorCode::ExternalLibraryFailure, "no depth buffer for an eye");
    }
    for (const std::uintptr_t native : images.textures) {
        const bgfx::TextureHandle colour =
            bgfx::createTexture2D(images.width_px, images.height_px, false, 1, images.format, k_target_flags,
                                  nullptr, static_cast<std::uint64_t>(native));
        if (!bgfx::isValid(colour)) {
            return core::fail(ErrorCode::ExternalLibraryFailure,
                              "an image of the headset could not be wrapped");
        }
        target.colours_.push_back(colour);
        const std::array<bgfx::TextureHandle, 2> attachments{colour, target.depth_};
        const bgfx::FrameBufferHandle frame_buffer =
            bgfx::createFrameBuffer(static_cast<std::uint8_t>(attachments.size()), attachments.data(), false);
        if (!bgfx::isValid(frame_buffer)) {
            return core::fail(ErrorCode::ExternalLibraryFailure, "no frame buffer for an eye");
        }
        target.frame_buffers_.push_back(frame_buffer);
    }
    return target;
}

EyeTarget::EyeTarget(EyeTarget&& other) noexcept
    : width_px_(other.width_px_), height_px_(other.height_px_),
      depth_(std::exchange(other.depth_, BGFX_INVALID_HANDLE)), colours_(std::move(other.colours_)),
      frame_buffers_(std::move(other.frame_buffers_)) {
    other.colours_.clear();
    other.frame_buffers_.clear();
}

EyeTarget& EyeTarget::operator=(EyeTarget&& other) noexcept {
    if (this != &other) {
        release();
        width_px_ = other.width_px_;
        height_px_ = other.height_px_;
        depth_ = std::exchange(other.depth_, BGFX_INVALID_HANDLE);
        colours_ = std::move(other.colours_);
        frame_buffers_ = std::move(other.frame_buffers_);
        other.colours_.clear();
        other.frame_buffers_.clear();
    }
    return *this;
}

EyeTarget::~EyeTarget() {
    release();
}

void EyeTarget::release() noexcept {
    for (bgfx::FrameBufferHandle& frame_buffer : frame_buffers_) {
        destroy_if_valid(frame_buffer);
    }
    for (bgfx::TextureHandle& colour : colours_) {
        destroy_if_valid(colour);
    }
    frame_buffers_.clear();
    colours_.clear();
    destroy_if_valid(depth_);
}

void EyeTarget::bind(bgfx::ViewId view, std::size_t image) const {
    if (image < frame_buffers_.size()) {
        bgfx::setViewFrameBuffer(view, frame_buffers_[image]);
    }
}

} // namespace helios::render::gpu
