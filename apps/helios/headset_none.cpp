// A build without VR (HELIOS_BUILD_XR off): there is no headset to open.
#include <utility>

#include "headset.hpp"

namespace helios::app {

struct Headset::State {};

core::Result<Headset> Headset::open() {
    return core::fail(core::ErrorCode::InvalidArgument, "this build has no VR support (HELIOS_BUILD_XR)");
}

Headset::Headset(std::unique_ptr<State> state) noexcept : state_(std::move(state)) {
}
Headset::Headset(Headset&& other) noexcept = default;
Headset& Headset::operator=(Headset&& other) noexcept = default;
Headset::~Headset() = default;

core::Result<bool> Headset::poll() {
    return false;
}

core::Result<xr::Frame> Headset::begin_frame() {
    return xr::Frame{};
}

render::gpu::ViewTarget Headset::target(const xr::Frame& /*frame*/, std::size_t /*eye*/,
                                        bgfx::ViewId view) const {
    return {.view_id = view};
}

core::VoidResult Headset::end_frame(const xr::Frame& /*frame*/) {
    return {};
}

double Headset::frame_period_s() const noexcept {
    return 0.0;
}

} // namespace helios::app
