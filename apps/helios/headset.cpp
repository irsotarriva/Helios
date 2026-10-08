#include "headset.hpp"

#include "helios/render/gpu/eye_target.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace helios::app {

using core::ErrorCode;

struct Headset::State {
    // The targets after the session: the wrappers go before the images they wrap.
    xr::Session session;
    std::vector<render::gpu::EyeTarget> targets;
};

core::Result<Headset> Headset::open() {
    if (bgfx::getRendererType() != bgfx::RendererType::Direct3D11) {
        return core::fail(ErrorCode::InvalidArgument, "VR needs the Direct3D 11 renderer (--renderer d3d11)");
    }
    auto session =
        xr::Session::make({.api = xr::GraphicsApi::Direct3D11, .device = bgfx::getInternalData()->context});
    if (!session) {
        return std::unexpected(session.error());
    }
    auto state = std::make_unique<State>(State{.session = std::move(*session), .targets = {}});
    for (const xr::EyeImages& images : state->session.images()) {
        constexpr std::uint32_t k_largest = std::numeric_limits<std::uint16_t>::max();
        if (images.width_px > k_largest || images.height_px > k_largest) {
            return core::fail(ErrorCode::OutOfRange, "the headset's images are too large");
        }
        auto target = render::gpu::EyeTarget::make({.width_px = static_cast<std::uint16_t>(images.width_px),
                                                    .height_px = static_cast<std::uint16_t>(images.height_px),
                                                    .format = images.format == xr::ColourFormat::Bgra8
                                                                  ? bgfx::TextureFormat::BGRA8
                                                                  : bgfx::TextureFormat::RGBA8,
                                                    .textures = images.textures});
        if (!target) {
            return std::unexpected(target.error());
        }
        state->targets.push_back(std::move(*target));
    }
    return Headset(std::move(state));
}

Headset::Headset(std::unique_ptr<State> state) noexcept : state_(std::move(state)) {
}
Headset::Headset(Headset&& other) noexcept = default;
Headset& Headset::operator=(Headset&& other) noexcept = default;

Headset::~Headset() {
    if (state_ != nullptr) {
        // Rationale: the session's images go with the session, and bgfx only lets go of its
        // wrappers of them when a frame is carried out.
        state_->targets.clear();
        bgfx::frame();
    }
}

core::Result<bool> Headset::poll() {
    return state_->session.poll();
}

core::Result<xr::Frame> Headset::begin_frame() {
    return state_->session.begin_frame();
}

render::gpu::ViewTarget Headset::target(const xr::Frame& frame, std::size_t eye, bgfx::ViewId view) const {
    const render::gpu::EyeTarget& images = state_->targets.at(eye);
    images.bind(view, frame.eyes.at(eye).image);
    return {.view_id = view, .width = images.width_px(), .height = images.height_px()};
}

core::VoidResult Headset::end_frame(const xr::Frame& frame) {
    return state_->session.end_frame(frame);
}

double Headset::frame_period_s() const noexcept {
    return state_->session.frame_period_s();
}

} // namespace helios::app
