#ifndef HELIOS_APPS_HELIOS_HEADSET_HPP
#define HELIOS_APPS_HELIOS_HEADSET_HPP

#include "helios/core/error.hpp"
#include "helios/render/gpu/map_renderer.hpp"
#include "helios/xr/session.hpp"

#include <bgfx/bgfx.h>
#include <cstddef>
#include <memory>

namespace helios::app {

// A headset for the client (BRIEFING D29): the OpenXR session and, for each eye, the images to
// draw into. Each frame: poll(), begin_frame(), draw into target() of each eye if the frame is
// to be drawn, bgfx::frame(), end_frame(). In a build without VR, open() fails and says so.
class Headset {
public:
    // After bgfx::init, on the renderer bgfx then uses (Direct3D 11 only, so far).
    [[nodiscard]] static core::Result<Headset> open();

    Headset(const Headset&) = delete;
    Headset& operator=(const Headset&) = delete;
    Headset(Headset&& other) noexcept;
    Headset& operator=(Headset&& other) noexcept;
    ~Headset();

    // False once the runtime wants the session over.
    [[nodiscard]] core::Result<bool> poll();
    // Waits for the headset's next frame: in VR this, not the window, paces the client.
    [[nodiscard]] core::Result<xr::Frame> begin_frame();
    // Points a bgfx view at the image of an eye for this frame.
    [[nodiscard]] render::gpu::ViewTarget target(const xr::Frame& frame, std::size_t eye,
                                                 bgfx::ViewId view) const;
    // After the bgfx::frame() that drew the eyes.
    [[nodiscard]] core::VoidResult end_frame(const xr::Frame& frame);

    [[nodiscard]] double frame_period_s() const noexcept;

private:
    struct State;
    explicit Headset(std::unique_ptr<State> state) noexcept;

    std::unique_ptr<State> state_;
};

} // namespace helios::app

#endif // HELIOS_APPS_HELIOS_HEADSET_HPP
